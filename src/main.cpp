#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include "secrets.h"
#include <Preferences.h>

// ---- RTSP (ESP32-RTSPServer) ----
// Library: https://github.com/rjsachse/ESP32-RTSPServer
#include <ESP32-RTSPServer.h>

// ---- Web + OTA ----
#include <WebServer.h>
#include <ArduinoOTA.h>

#include <sys/time.h>
#include <math.h>
#include <string.h>
#include "esp_sntp.h"
#include "favicon.h"

// ---- Camera pin map for AI Thinker ESP32-CAM ----
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_PIN            4

// ---- Chatty Serial ----
#define API_VERBOSE_SERIAL 1    // set to 0 to silence API call logs

// =============================================================
//  GLOBALS
// =============================================================
Preferences prefs;
Preferences camPrefs;
WiFiClient netClient;
PubSubClient mqtt(netClient);
WebServer web(80);

// Nominal “status” resolution for RTSP (may differ from actual fb->width/height)
static const uint16_t STREAM_WIDTH  = 640;
static const uint16_t STREAM_HEIGHT = 480;

// ESP32-RTSPServer instance (video-only for now)
static RTSPServer rtspServer;

// RTSP stream path
static const char *RTSP_STREAM_PATH = "mjpeg";   // Changeable


bool  stream_on   = false;
bool  led_active  = false;
unsigned long led_on_ms           = 0;
unsigned long last_telem_ms       = 0;
unsigned long last_mqtt_attempt_ms = 0;

// Web UI options
static bool show_fahrenheit   = false;   // toggled at /toggle_temp
static bool stream_default_on = true;    // default stream-on state persisted in prefs

// Don't call OTA when disabled
static bool ota_enabled = false;

// =============================================================
//  TELEMETRY / TIMING CONSTANTS
// =============================================================
static const uint32_t TELEMETRY_INTERVAL_MS   = 15000;
static const uint32_t MQTT_RETRY_INTERVAL_MS  = 5000;
static const uint32_t FLASH_AUTO_OFF_MS       = 10000;

// =============================================================
//  ArduinoOTA Setup
// =============================================================
static void setupOTA() {
    // --------------------------------------------------------
    // Determine if OTA should be enabled
    // OTA is disabled if hostname or IP are missing.
    // This makes OTA opt-in and prevents accidental exposure.
    // --------------------------------------------------------
    if (strlen(OTA_HOSTNAME) == 0) {
        Serial.println("[OTA] Disabled (missing hostname)");
        return;
    }

    // --------------------------------------------------------
    // OTA Enabled — configure normally
    // --------------------------------------------------------
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    if (strlen(OTA_PASSWORD) > 0) {
        ArduinoOTA.setPassword(OTA_PASSWORD);
    }

    uint16_t port = 3232;  // default
    if (strlen(OTA_PORT) > 0) {
        port = atoi(OTA_PORT);
    }
    ArduinoOTA.setPort(port);

    // Optional diagnostics
    ArduinoOTA.onStart([]() {
        Serial.println("[OTA] Start");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("[OTA] End");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error: %u\n", error);
    });

    ArduinoOTA.begin();
    Serial.printf("[OTA] Ready on %s:%u\n", OTA_HOSTNAME, port);
    ota_enabled = true;
}

// =============================================================
//  TEMP SENSOR
// =============================================================
extern "C" uint8_t temprature_sens_read();
static float readCpuTempC() {
  return (125.0f * ((float)temprature_sens_read() / 255.0f)) - 40.0f;
}
static float readCcdTempC() {
  // Not easily available on ESP32-CAM without external sensor
  return NAN;
}

// =============================================================
//  LOGGING HELPERS
// =============================================================

static void log_line(const char* msg, bool verbose = false) {
  Serial.println(msg);
  if (!mqtt.connected()) return;

#ifdef MQTT_TOPIC_VERBOSE
  if (verbose) {
    mqtt.publish(MQTT_TOPIC_VERBOSE, msg, false);
    return;
  }
#endif
  mqtt.publish(MQTT_TOPIC_STATUS, msg, false);
}

static void logf(const char* fmt, ...) {
  char buf[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  log_line(buf, true);
}

static void publish_verbose(const char* msg) {
  log_line(msg, true);
}

static void api_log(const char* msg) {
#if API_VERBOSE_SERIAL
    Serial.println(msg);
#endif
}

// =============================================================
//  CAMERA AUTODETECT
// =============================================================
static esp_err_t camera_reinit(uint32_t xclk_hz, framesize_t fsize, int jpeg_quality, int fb_count) {
  camera_config_t config;
  memset(&config, 0, sizeof(config));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = xclk_hz;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = fsize;
  config.jpeg_quality = jpeg_quality;
  config.fb_count     = fb_count;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  esp_camera_deinit();
  return esp_camera_init(&config);
}

static bool camera_init_auto() {
  int fb_count = psramFound() ? 2 : 1;

  // Prefer QVGA-ish to keep bandwidth and RAM usage modest; framesize still VGA for RTSP
  if (camera_reinit(20000000, FRAMESIZE_VGA, 10, fb_count) != ESP_OK) {
    if (camera_reinit(10000000, FRAMESIZE_QVGA, 12, fb_count) != ESP_OK) {
      Serial.println("Camera init failed (both attempts).");
      return false;
    }
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("No sensor handle.");
    return false;
  }

  Serial.printf("Detected camera PID: 0x%04x\n", s->id.PID);

  // Basic tweaks: flip image vertically for ESP32-CAM orientation
  s->set_vflip(s, 1);
  s->set_hmirror(s, 0);

  return true;
}

// =============================================================
//  OV2640 RAW TEMPERATURE REGISTER (UNOFFICIAL)
//  NOTE: Must restore sensor registers after reading to avoid
//  corrupting normal streaming configuration.
// =============================================================
static int read_ov2640_temp_raw() {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return -1;

    // Select sensor register bank 1
    if (s->set_reg(s, 0xFF, 0x01, 0x01) != 0) {
        return -2;
    }

    // Save original value of reg 0x12 in bank 1
    int reg12_old = s->get_reg(s, 0x12, 0xFF);

    // Enable ADC mode (as per some OV2640 notes)
    s->set_reg(s, 0x12, 0x04, 0x04);

    // Read ADC register (values vary by module)
    int val = s->get_reg(s, 0x0A, 0xFF);

    // Restore original reg 0x12
    s->set_reg(s, 0x12, 0xFF, reg12_old);

    // Switch back to sensor register bank 0 (normal operation)
    s->set_reg(s, 0xFF, 0x01, 0x00);

    return val;
}

// =============================================================
//  TELEMETRY JSON BUILDER
// =============================================================
static void build_status_json(char* out, size_t out_len) {
  float cpuC = readCpuTempC();
  float cpuF = cpuC * 9.0f / 5.0f + 32.0f;
  float ccdC = readCcdTempC();
  float ccdF = ccdC * 9.0f / 5.0f + 32.0f;

  unsigned long uptime_s  = millis() / 1000UL;
  unsigned long esp_epoch = (unsigned long)time(nullptr);

  int rssi = WiFi.RSSI();
  uint32_t heap_free  = ESP.getFreeHeap();
  uint32_t psram_free = ESP.getFreePsram();

  char ccdC_field[24];
  char ccdF_field[24];

  if (isnan(ccdC)) {
    strcpy(ccdC_field, "null");
    strcpy(ccdF_field, "null");
  } else {
    dtostrf(ccdC, 0, 1, ccdC_field);
    dtostrf(ccdF, 0, 1, ccdF_field);
  }

  IPAddress ip = WiFi.localIP();

  snprintf(
    out,
    out_len,
    "{"
      "\"device\":\"%s\","
      "\"ip\":\"%s\","
      "\"uptime_s\":%lu,"
      "\"esp_time\":%lu,"
      "\"rssi_dbm\":%d,"
      "\"heap_free\":%u,"
      "\"psram_free\":%u,"
      "\"cpu_temp_c\":%.1f,"
      "\"cpu_temp_f\":%.1f,"
      "\"ccd_temp_c\":%s,"
      "\"ccd_temp_f\":%s,"
      "\"stream_on\":%s,"
      "\"flash_on\":%s"
    "}",
    DEVICE_NAME,
    ip.toString().c_str(),
    uptime_s,
    esp_epoch,
    rssi,
    heap_free,
    psram_free,
    cpuC,
    cpuF,
    ccdC_field,
    ccdF_field,
    stream_on ? "true" : "false",
    led_active ? "true" : "false"
  );
}

// MQTT telemetry publisher (compact JSON)
static void publish_telemetry() {
  if (!mqtt.connected()) return;
  char msg[256];
  build_status_json(msg, sizeof(msg));
  mqtt.publish(MQTT_TOPIC_TELEM, msg, true);
}

// Simple status text message
static void publish_status(const char* msg) {
  if (!mqtt.connected()) {
    Serial.print("[STATUS] (offline MQTT) ");
    Serial.println(msg);
    return;
  }
  mqtt.publish(MQTT_TOPIC_STATUS, msg, true);
}

// =============================================================
//  CONTROL HELPERS (STREAM / FLASH)
// =============================================================
static void set_stream(bool on) {
  if (stream_on == on) return;
  stream_on = on;
  publish_status(on ? "stream:on" : "stream:off");
  logf("Stream %s", on ? "ENABLED" : "DISABLED");
}

static void set_flash(uint8_t value) {
  ledcWrite(0, value);
  bool now_on = (value > 0);
  if (now_on) {
    led_active = true;
    led_on_ms = millis();
    publish_status("flash:on");
  } else {
    led_active = false;
    publish_status("flash:off");
  }
}

// =============================================================
//  MQTT HANDLING
// =============================================================
static void mqtt_callback(char* topic, byte* payload, unsigned int len) {
  String cmd;
  cmd.reserve(len);
  for (unsigned int i = 0; i < len; ++i) {
    cmd += static_cast<char>(payload[i]);
  }
  cmd.toLowerCase();

  if (cmd.indexOf("start") != -1) {
    set_stream(true);
  } else if (cmd.indexOf("stop") != -1) {
    set_stream(false);
  } else if (cmd.startsWith("flash:")) {
    int val = constrain(cmd.substring(6).toInt(), 0, 255);
    set_flash((uint8_t)val);
  }
}

static void mqtt_connect_once() {
  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  log_line("MQTT: attempting connection...", true);

  bool ok = false;
#if defined(MQTT_USER) && defined(MQTT_PASS)
  ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
#else
  ok = mqtt.connect(MQTT_CLIENT_ID);
#endif

  if (!ok) {
    int8_t state = mqtt.state();
    char buf[96];
    snprintf(buf, sizeof(buf), "MQTT connect failed, state=%d", state);
    log_line(buf, true);
    return;
  }

  log_line("MQTT connected", true);
  mqtt.subscribe(MQTT_TOPIC_CMD);
  publish_status("online");

  // Announce RTSP URL
  char rtsp_url[96];
  snprintf(rtsp_url, sizeof(rtsp_url), "rtsp://%s:%d/",
    WiFi.localIP().toString().c_str(), RTSP_PORT);
  mqtt.publish(MQTT_TOPIC_STATUS, rtsp_url, true);
}

// =============================================================
//  WEB HELPERS
// =============================================================
static void redirect_home() {
  web.sendHeader("Location", "/");
  web.send(303);
}

// HTML root page
static void handle_root() {
  String html;
  html.reserve(4000);

  IPAddress ip = WiFi.localIP();
  unsigned long uptime_s = millis() / 1000UL;
  int rssi = WiFi.RSSI();
  uint32_t heap_free  = ESP.getFreeHeap();
  uint32_t psram_free = ESP.getFreePsram();
  float cpuC = readCpuTempC();
  float cpuF = cpuC * 9.0f / 5.0f + 32.0f;

  time_t now_ts = time(nullptr);
  struct tm tm_info;
  localtime_r(&now_ts, &tm_info);

  char time_buf[40];
  strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S %Z", &tm_info);

  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>ESP32-CAM - ");
  html += DEVICE_NAME;
  html += F("</title>"
            "<link rel='icon' type='image/png' href='/favicon.ico'>"
            "<style>"
            "body{margin:0;padding:0;font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:#111;color:#eee}"
            "header{background:#222;padding:10px 16px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid #333}"
            "header h1{margin:0;font-size:18px}"
            "header span{font-size:12px;color:#aaa}"
            "main{padding:12px 16px;display:flex;flex-direction:column;gap:12px}"
            ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}"
            ".card{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:10px 12px;box-sizing:border-box}"
            ".card h2{margin:0 0 4px 0;font-size:14px;color:#f0f0f0}"
            ".label{font-size:11px;color:#888}"
            ".value{font-size:13px}"
            // Unified two-column rows
            ".row{"
              "display:flex;"
              "flex-wrap:wrap;"
              "justify-content:space-between;"
              "margin-bottom:4px;"
            "}"
            ".col{"
              "flex:1 1 calc(50% - 8px);"
              "margin:2px 0;"
            "}"
            "button, .btn{display:inline-block;padding:6px 10px;border-radius:6px;border:1px solid #444;background:#2b2b2b;color:#eee;font-size:12px;text-decoration:none;cursor:pointer;margin:2px 2px 0 0}"
            "button:hover,.btn:hover{background:#3b3b3b}"
            "img{max-width:100%;height:auto;border-radius:6px;border:1px solid #333}"
            "code{font-size:11px;background:#000;padding:2px 4px;border-radius:4px}"

            // Camera controls layout
            ".cam-card{display:flex;flex-direction:column;}"
            "#cam_ctrls{flex:1;display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));"
            "gap:4px 12px;align-content:flex-start;}"
            ".cam-group-title{grid-column:1/-1;margin-top:6px;font-size:11px;color:#aaa;"
            "text-transform:uppercase;letter-spacing:0.04em;}"
            ".cam-footer{margin-top:8px;display:flex;justify-content:flex-end;}"
            "</style>"
            "<script>"
            "const tzMap={\"America/Los_Angeles\":\"PST8PDT,M3.2.0/2,M11.1.0/2\"};"
            "let lastAutoSync=0;"

            "async function syncClock(){"
              "const epoch=Math.floor(Date.now()/1000);"
              "const browserIANA=Intl.DateTimeFormat().resolvedOptions().timeZone;"
              "const posix=tzMap[browserIANA]||'UTC0';"
              "await fetch('/api/sync_clock?epoch='+epoch+'&tz='+encodeURIComponent(posix));"
              "await refreshStatus();"
            "}"

            "async function refreshStatus(){"
              "try{"
                "const r=await fetch('/api/status');"
                "if(!r.ok)return;"
                "const j=await r.json();"
                "document.getElementById('uptime').textContent=j.uptime_s+' s';"
                "document.getElementById('rssi').textContent=j.rssi_dbm+' dBm';"
                "document.getElementById('heap').textContent=j.heap_free+' B';"
                "document.getElementById('psram').textContent=j.psram_free+' B';"
                "document.getElementById('stream_state').textContent=j.stream_on?'ON':'OFF';"
                "document.getElementById('flash_state').textContent=j.flash_on?'ON':'OFF';"
                "if(document.body.dataset.tempFormat==='F'){"
                  "document.getElementById('cpu_temp_display').textContent=j.cpu_temp_f.toFixed(1)+' °F';"
                "}else{"
                  "document.getElementById('cpu_temp_display').textContent=j.cpu_temp_c.toFixed(1)+' °C';"
                "}"
                "fetch('/ccd_raw').then(r=>r.text()).then(val=>{"
                  "const el=document.getElementById('ccd_raw');"
                  "if(el)el.textContent=val;"
                "}).catch(()=>{});"
            "if(j.esp_time!==undefined){"
              "const espDate=new Date(j.esp_time*1000);"
              "const browserDate=new Date();"
              "document.getElementById('esp_time_display').textContent=espDate.toLocaleString();"
              "document.getElementById('browser_time').textContent=browserDate.toLocaleString();"
              "const delta=(Date.now()/1000 - j.esp_time);"
              "document.getElementById('time_delta').textContent=delta.toFixed(1)+' s';"
              "document.getElementById('cur_time').textContent=espDate.toLocaleString();"

              // ---- Auto clock sync logic ----
              "const absDelta=Math.abs(delta);"
              "const nowMs=Date.now();"
              // Only auto-sync if drift is > 2s and we haven't auto-synced recently
              "if(absDelta>2){"
                "if(nowMs - lastAutoSync > 60000){"  // 60s cooldown
                  "lastAutoSync=nowMs;"
                  "syncClock();"
                "}"
              "}"
            "}"
              "}catch(e){}"
            "}"

            "function refreshSnap(){"
              "const img=document.getElementById('snap');"
              "if(!img)return;"
              "img.src='/snapshot.jpg?ts='+Date.now();"
            "}"

            "async function loadSettings(){"
              "try{"
                "const r=await fetch('/api/get_settings');"
                "if(!r.ok)return;"
                "const j=await r.json();"
                "document.getElementById('tz_display').textContent=j.timezone||'UTC0';"
                "if(j.temp_format==='F'){"
                  "document.body.dataset.tempFormat='F';"
                  "document.getElementById('temp_mode_display').textContent='Fahrenheit';"
                "}else{"
                  "document.body.dataset.tempFormat='C';"
                  "document.getElementById('temp_mode_display').textContent='Celsius';"
                "}"
                "document.getElementById('stream_default_display').textContent=j.stream_on?'ON':'OFF';"
              "}catch(e){}"
            "}"

            "async function setBrowserTZ(){"
              "const browserIANA=Intl.DateTimeFormat().resolvedOptions().timeZone;"
              "const posix=tzMap[browserIANA]||'UTC0';"
              "await fetch('/api/set_tz?tz='+encodeURIComponent(posix));"
              "await loadSettings();"
            "}"

            "async function toggleTempMode(){"
              "await fetch('/toggle_temp');"
              "await loadSettings();"
              "await refreshStatus();"
            "}"

            "async function toggleStreamDefault(){"
              "await fetch('/api/toggle_stream_default');"
              "await loadSettings();"
            "}"

            "async function loadCameraControls(){"
              "try{"
                "const r=await fetch('/api/cam_settings');"
                "if(!r.ok)return;"
                "const s=await r.json();"
                "let html='';"

                "function addSlider(name,label,min,max){"
                  "html+='<div class=\"label\">'+label+'</div>';"
                  "html+='<input type=\"range\" min=\"'+min+'\" max=\"'+max+'\" value=\"'+s[name]+'\" id=\"ctl_'+name+'\">';"
                "}"

                "function addToggle(name,label){"
                  "const checked=s[name]?'checked':'';"
                  "html+='<div class=\"label\">'+label+'</div>';"
                  "html+='<input type=\"checkbox\" id=\"ctl_'+name+'\" '+checked+'>';"
                "}"

                // Exposure
                "html+='<div class=\"cam-group-title\">Exposure</div>';"
                "addSlider('ae_level','AE level',-2,2);"
                "addSlider('aec_value','AEC value',0,1200);"
                "addSlider('agc_gain','AGC gain',0,30);"
                "addToggle('aec2','AEC2');"

                // Color
                "html+='<div class=\"cam-group-title\">Color</div>';"
                "addSlider('brightness','Brightness',-2,2);"
                "addSlider('contrast','Contrast',-2,2);"
                "addSlider('saturation','Saturation',-2,2);"
                "addSlider('denoise','Denoise',0,8);"
                "addToggle('awb','AWB');"
                "addToggle('awb_gain','AWB gain');"

                // Geometry
                "html+='<div class=\"cam-group-title\">Geometry</div>';"
                "addToggle('hmirror','Horizontal mirror');"
                "addToggle('vflip','Vertical flip');"

                // Quality
                "html+='<div class=\"cam-group-title\">Quality</div>';"
                "addSlider('sharpness','Sharpness',-3,3);"
                "addSlider('quality','JPEG quality',5,63);"
                "html+='<div class=\"label\">Framesize</div>';"
                "html+='<select id=\"ctl_framesize\">';"
                "const fsOptions=[0,1,2,3,4,5,6,7,8,9];"  // 5=QVGA / 8=VGA / 9=SVGA 
                "for(let i=0;i<fsOptions.length;i++){"
                  "const f=fsOptions[i];"
                  "const sel=(s.framesize==f)?' selected':'';"
                  "html+='<option value=\"'+f+'\"'+sel+'>'+f+'</option>';"
                "}"
                "html+='</select>';"

                "document.getElementById('cam_ctrls').innerHTML=html;"
              "}catch(e){console.log('cam ctrl error',e);}"
            "}"

            "async function applyCameraSettings(){"
              "let payload={};"
              "function grab(n){"
                 "let el=document.getElementById('ctl_'+n);"
                 "if(!el)return;"
                 "payload[n]=(el.type==='checkbox')?(el.checked?1:0):parseInt(el.value);"
              "}"
              "grab('brightness');"
              "grab('contrast');"
              "grab('saturation');"
              "grab('sharpness');"
              "grab('denoise');"
              "grab('ae_level');"
              "grab('aec_value');"
              "grab('agc_gain');"
              "grab('aec2');"
              "grab('awb');"
              "grab('awb_gain');"
              "grab('hmirror');"
              "grab('vflip');"
              "grab('quality');"
              "let fs=document.getElementById('ctl_framesize');"
              "payload['framesize']=parseInt(fs.value);"
              "await fetch('/api/set_cam_params',{"
                "method:'POST',"
                "headers:{'Content-Type':'application/json'},"
                "body:JSON.stringify(payload)"
              "});"
              "loadCameraControls();"
            "}"

            "async function setCamParam(param,value){"
              "await fetch('/api/set_cam_param?param='+param+'&value='+value);"
              "loadCameraControls();"
            "}"

            "function setFlash(val){"
              "fetch('/flash?val='+val).then(()=>refreshStatus());"
            "}"
            "function startStream(){fetch('/start').then(()=>refreshStatus());}"
            "function stopStream(){fetch('/stop').then(()=>refreshStatus());}"
            "window.addEventListener('load',()=>{"
              "loadSettings();"
              "refreshStatus();"
              "refreshSnap();"
              "loadCameraControls();"
            "});"
            "setInterval(refreshStatus,5000);"
            "setInterval(refreshSnap,3000);"
            "</script>"
            "</head><body>");

  html += F("<header><div><h1>ESP32-CAM ");
  html += DEVICE_NAME;
  html += F("</h1><span>");
  html += ip.toString();
  html += F("</span></div><div><span class='label'>CPU temp: </span><span class='value'>");
  if (show_fahrenheit) {
    html += String(cpuF, 1);
    html += F(" °F");
  } else {
    html += String(cpuC, 1);
    html += F(" °C");
  }
  html += F("</span></div></header>");

  html += F("<main><div class='grid'>");

  // System card
  html += F("<div class='card'><h2>System</h2>");

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>Device</div><div class='value'>" DEVICE_NAME "</div></div>"
      "<div class='col'><div class='label'>IP</div><div class='value'>" + ip.toString() + "</div></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>Uptime</div><div class='value' id='uptime'>" + String(uptime_s) + " s</div></div>"
      "<div class='col'><div class='label'>WiFi RSSI</div><div class='value' id='rssi'>" + String(rssi) + " dBm</div></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>Heap free</div><div class='value' id='heap'>" + String(heap_free) + " B</div></div>"
      "<div class='col'><div class='label'>PSRAM free</div><div class='value' id='psram'>" + String(psram_free) + " B</div></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>Timezone</div><div class='value' id='tz_display'>--</div></div>"
      "<div class='col'><button onclick='setBrowserTZ()'>Use Browser Timezone</button></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>ESP32 Time</div><div class='value' id='esp_time_display'>" + String(time_buf) + "</div></div>"
      "<div class='col'><div class='label'>Browser Time</div><div class='value' id='browser_time'>--</div></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>Delta (Browser - ESP32)</div><div class='value' id='time_delta'>--</div></div>"
      "<div class='col'><div class='label'>Current Time (ESP)</div><div class='value' id='cur_time'>" + String(time_buf) + "</div></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>CPU Temp</div><div class='value' id='cpu_temp_display'>" +
        (show_fahrenheit ? String(cpuF,1)+" °F" : String(cpuC,1)+" °C") +
      "</div></div>"
      "<div class='col'><div class='label'>CCD Raw</div><div class='value' id='ccd_raw'>" + String(read_ov2640_temp_raw()) + "</div></div>"
    "</div>";

  html +=
    "<div style='margin-top:6px;'>"
      "<button onclick='syncClock()'>Sync Clock (with TZ)</button>"
    "</div>";

  html += F("</div>");

  // Snapshot card
  html += F("<div class='card'><h2>Snapshot</h2>"
            "<div class='label'>Preview (QVG-ish)</div>"
            "<img id='snap' src='/snapshot.jpg' alt='snapshot'>"
            "</div>");

  // Settings card
  html += F("<div class='card'><h2>Settings</h2>");

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>Temperature Mode</div><div class='value' id='temp_mode_display'>--</div></div>"
      "<div class='col'><button onclick='toggleTempMode()'>Toggle C/F</button></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>Stream Default</div><div class='value' id='stream_default_display'>--</div></div>"
      "<div class='col'><button onclick='toggleStreamDefault()'>Toggle Stream Default</button></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><div class='label'>Stream state</div><div class='value' id='stream_state'>" +
        String(stream_on ? "ON" : "OFF") + "</div></div>"
      "<div class='col'><button onclick='applyCamDefaults()'>Reset Cam Defaults</button></div>"
    "</div>";

  html +=
    "<div class='row' style='margin-top:6px;'>"
      "<div class='col'><button onclick='startStream()'>Start Stream</button></div>"
      "<div class='col'><button onclick='stopStream()'>Stop Stream</button></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><button onclick='setFlash(0)'>Flash Off</button></div>"
      "<div class='col'><button onclick='setFlash(64)'>Flash Low</button></div>"
    "</div>";

  html +=
    "<div class='row'>"
      "<div class='col'><button onclick='setFlash(255)'>Flash High</button></div>"
      "<div class='col'></div>"
    "</div>";

  html += F("</div>"); // end Settings card

  // Camera Controls card
  html += F("<div class='card cam-card'><h2>Camera Controls</h2>"
            "<div id='cam_ctrls'></div>"
            "<div class='cam-footer'>"
              "<button onclick='applyCameraSettings()'>Apply Camera Settings</button>"
            "</div>"
            "</div>");

  html += F("</div>");

  html += F("<div class='card' style='margin-top:12px;'>"
            "<h2>API</h2>"
            "<div class='label'>Status JSON</div>"
            "<div class='value'><code>GET /api/status</code></div>"
            "<div class='label'>Snapshot</div>"
            "<div class='value'><code>GET /snapshot.jpg</code></div>"
            "<div class='label'>Control</div>"
            "<div class='value'><code>GET /api/start</code>, <code>/api/stop</code>, "
            "<code>/api/flash?val=0-255</code></div>"
            "<div class='label'>Time/Timezone</div>"
            "<div class='value'><code>POST /api/set_tz?tz=...</code>, "
            "<code>POST /api/sync_clock?epoch=...&tz=...</code></div>"
            "</div>");

  html += F("</main></body></html>");

  web.send(200, "text/html", html);
}

// /ccd_raw – expose raw OV2640 temperature register
static void handle_ccd_raw() {
  int val = read_ov2640_temp_raw();
  web.send(200, "text/plain", String(val));
}

// /snapshot.jpg (single frame)
static void handle_snapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    web.send(503, "text/plain", "Camera busy");
    return;
  }
  web.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  web.sendHeader("Pragma", "no-cache");
  web.sendHeader("Expires", "0");
  web.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// /api/status JSON
static void handle_api_status() {
  char json[256];
  build_status_json(json, sizeof(json));
  web.send(200, "application/json", json);
}

// /sync?epoch=... (legacy/manual)
static void handle_sync() {
  if (web.hasArg("epoch")) {
    time_t t = (time_t)web.arg("epoch").toInt();
    struct timeval tv;
    tv.tv_sec = t;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    log_line("Clock synchronized from /sync", true);
  }
  redirect_home();
}

// API versions of start/stop/flash (JSON, no redirect)
static void handle_api_start() {
  set_stream(true);
  web.send(200, "application/json", "{\"ok\":true,\"stream_on\":true}");
}

static void handle_api_stop() {
  set_stream(false);
  web.send(200, "application/json", "{\"ok\":true,\"stream_on\":false}");
}

static void handle_api_flash() {
  int val = 0;
  if (web.hasArg("val")) {
    val = constrain(web.arg("val").toInt(), 0, 255);
  }
  set_flash((uint8_t)val);
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"flash\":%d}", val);
  web.send(200, "application/json", buf);
}

// =============================================================
//  CAMERA SETTINGS PERSISTENCE
// =============================================================
static void apply_saved_camera_settings() {
    camPrefs.begin("cam", true);  // read-only

    sensor_t* s = esp_camera_sensor_get();
    if (!s) {
        camPrefs.end();
        return;
    }

    // macro to reduce boilerplate
    auto loadInt = [&](const char* key, int def) {
        return camPrefs.getInt(key, def);
    };

    // Exposure
    s->set_ae_level(s, loadInt("ae_level",      s->status.ae_level));
    s->set_aec_value(s, loadInt("aec_value",    s->status.aec_value));
    s->set_agc_gain(s, loadInt("agc_gain",      s->status.agc_gain));
    s->set_aec2(s,      loadInt("aec2",         s->status.aec2));

    // Color
    s->set_brightness(s, loadInt("brightness",  s->status.brightness));
    s->set_contrast(s,   loadInt("contrast",    s->status.contrast));
    s->set_saturation(s, loadInt("saturation",  s->status.saturation));
    s->set_denoise(s,    loadInt("denoise",     s->status.denoise));
    s->set_whitebal(s,   loadInt("awb",         s->status.awb));
    s->set_awb_gain(s,   loadInt("awb_gain",    s->status.awb_gain));

    // Geometry
    s->set_hmirror(s, loadInt("hmirror", s->status.hmirror));
    s->set_vflip(s,   loadInt("vflip",   s->status.vflip));

    // Quality
    s->set_sharpness(s, loadInt("sharpness", s->status.sharpness));
    s->set_quality(s,   loadInt("quality",   s->status.quality));

    int fs = loadInt("framesize", s->status.framesize);
    s->set_framesize(s, (framesize_t)fs);

    camPrefs.end();
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  publish_verbose("Boot start");

  Serial.println();
  Serial.println("=== ESP32-CAM Boot ===");
  Serial.printf("Device: %s\n", DEVICE_NAME);
  Serial.printf("Chip: %s  rev:%d  CPU:%dMHz\n",
                ESP.getChipModel(),
                ESP.getChipRevision(),
                ESP.getCpuFreqMHz());
  Serial.printf("Heap: %u free / %u total,  PSRAM: %u free / %u total\n",
                ESP.getFreeHeap(), ESP.getHeapSize(),
                ESP.getFreePsram(), ESP.getPsramSize());

  // Non-Volatile Settings
  prefs.begin("settings", false);
  String tz = prefs.getString("timezone", "UTC0");  // default UTC
  setenv("TZ", tz.c_str(), 1);
  tzset();
  Serial.printf("Loaded TZ: %s\n", tz.c_str());

  // Load UI-related preferences
  show_fahrenheit   = prefs.getBool("tempF", false);
  stream_default_on = prefs.getBool("stream_default", true);
  Serial.printf("Loaded tempF=%s, stream_default=%s\n",
                show_fahrenheit ? "true" : "false",
                stream_default_on ? "true" : "false");

  // Camera
  if (!camera_init_auto()) {
    Serial.println("Camera init failed, halting.");
    while (true) delay(1000);
  }
  Serial.println("Camera initialized.");
  apply_saved_camera_settings();
  Serial.println("Loaded saved camera settings.");

  // --------------------------------------------------------
  // WiFi (must be initialized BEFORE any network servers/OTA)
  // --------------------------------------------------------
  WiFi.mode(WIFI_STA);

  if (strlen(STA_IP) > 0 && strlen(STA_GATEWAY) > 0 && strlen(STA_SUBNET) > 0) {
    IPAddress ip, gw, sn, dns;
    ip.fromString(STA_IP);
    gw.fromString(STA_GATEWAY);
    sn.fromString(STA_SUBNET);
    if (strlen(STA_DNS) > 0) dns.fromString(STA_DNS);
    else dns = gw;

    if (!WiFi.config(ip, gw, sn, dns)) {
      Serial.println("Static IP config failed, using DHCP.");
    } else {
      Serial.printf("Static IP configured: %s\n", ip.toString().c_str());
    }
  }

  Serial.printf("Connecting to WiFi SSID '%s'...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
    if (millis() - wifi_start > 15000) {
      Serial.println("\nWiFi connect timeout. Rebooting.");
      ESP.restart();
    }
  }
  Serial.printf("\nWiFi connected. IP: %s  RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  // --------------------------------------------------------
  // Stop SNTP from overwriting manually-set browser time
  // --------------------------------------------------------
  bool noGateway = (strlen(STA_GATEWAY) == 0);
  bool noDNS     = (strlen(STA_DNS) == 0);

  if (noGateway || noDNS) {
      Serial.println("No gateway/DNS -> forcing manual clock mode");
      sntp_stop();
      sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
      configTime(0, 0, nullptr, nullptr);
  } else {
      Serial.println("Gateway + DNS present -> enabling SNTP");
      configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  }

  // MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqtt_callback);
  mqtt_connect_once();  // one attempt at boot; loop() will retry later

  // LED (flash) PWM
  ledcSetup(0, 5000, 8);
  ledcAttachPin(LED_PIN, 0);
  set_flash(0);

  // --------------------------------------------------------
  // RTSP server (MUST come AFTER WiFi + camera are initialized)
  // --------------------------------------------------------
  rtspServer.transport = RTSPServer::VIDEO_ONLY; // video-only for now
  rtspServer.rtspPort = RTSP_PORT; // from secrets.h
  rtspServer.maxRTSPClients = 3; // small, sane default

  // Optional auth if you want it later:
  // rtspServer.setCredentials("user", "pass");

  bool ok = rtspServer.init(
  RTSPServer::VIDEO_ONLY, // transport
  RTSP_PORT, // RTSP port
  0, // sampleRate (0 = no audio)
  0, 0, 0, // ports (0 = use defaults from class)
  IPAddress(), // RTP IP (0 = default)
  255 // TTL (255 = default per header)
  );

  if (ok) {
  Serial.printf("RTSP server started on port %d\n", RTSP_PORT);
  } else {
  Serial.println("ERROR: RTSP server failed to start");
  }

  // --------------------------------------------------------
  // OTA
  // --------------------------------------------------------
  setupOTA();

  // --------------------------------------------------------
  // Web routes
  // --------------------------------------------------------
  web.on("/", HTTP_GET, handle_root);
  web.on("/snapshot.jpg", HTTP_GET, handle_snapshot);
  web.on("/api/status", HTTP_GET, handle_api_status);

  web.on("/api/cam_settings", HTTP_GET, []() {
      api_log("API /api/cam_settings called");

      sensor_t* s = esp_camera_sensor_get();
      if (!s) {
          web.send(500, "application/json", "{\"error\":\"no sensor\"}");
          return;
      }

      char json[640];  // bigger buffer for more fields

      snprintf(json, sizeof(json),
          "{"
            "\"brightness\":%d,"
            "\"contrast\":%d,"
            "\"saturation\":%d,"
            "\"sharpness\":%d,"
            "\"denoise\":%d,"

            "\"aec2\":%d,"
            "\"aec_value\":%d,"
            "\"ae_level\":%d,"
            "\"agc_gain\":%d,"

            "\"awb\":%d,"
            "\"awb_gain\":%d,"
            "\"wpc\":%d,"
            "\"raw_gma\":%d,"

            "\"gainceiling\":%d,"
            "\"quality\":%d,"
            "\"framesize\":%d,"

            "\"hmirror\":%d,"
            "\"vflip\":%d"

          "}",
          s->status.brightness,
          s->status.contrast,
          s->status.saturation,
          s->status.sharpness,
          s->status.denoise,

          s->status.aec2,
          s->status.aec_value,
          s->status.ae_level,
          s->status.agc_gain,

          s->status.awb,
          s->status.awb_gain,
          s->status.wpc,
          s->status.raw_gma,

          s->status.gainceiling,
          s->status.quality,
          s->status.framesize,

          s->status.hmirror,
          s->status.vflip
      );

      web.send(200, "application/json", json);
  });

  web.on("/ccd_raw", HTTP_GET, handle_ccd_raw);

  // Apply multiple camera parameters
  web.on("/api/set_cam_params", HTTP_POST, []() {
      if (!web.hasArg("plain")) {
          web.send(400, "application/json", "{\"error\":\"missing json\"}");
          return;
      }

      String body = web.arg("plain");
      StaticJsonDocument<512> doc;
      DeserializationError err = deserializeJson(doc, body);
      if (err) {
          web.send(400, "application/json", "{\"error\":\"bad json\"}");
          return;
      }

      sensor_t* s = esp_camera_sensor_get();

      // Apply in safest order
      if (doc.containsKey("aec2"))      s->set_aec2(s, doc["aec2"]);
      if (doc.containsKey("awb"))       s->set_whitebal(s, doc["awb"]);
      if (doc.containsKey("awb_gain"))  s->set_awb_gain(s, doc["awb_gain"]);
      if (doc.containsKey("agc_gain"))  s->set_agc_gain(s, doc["agc_gain"]);
      if (doc.containsKey("aec_value")) s->set_aec_value(s, doc["aec_value"]);
      if (doc.containsKey("ae_level"))  s->set_ae_level(s, doc["ae_level"]);
      if (doc.containsKey("sharpness")) s->set_sharpness(s, doc["sharpness"]);
      if (doc.containsKey("denoise"))   s->set_denoise(s, doc["denoise"]);
      if (doc.containsKey("brightness"))s->set_brightness(s, doc["brightness"]);
      if (doc.containsKey("contrast"))  s->set_contrast(s, doc["contrast"]);
      if (doc.containsKey("saturation"))s->set_saturation(s, doc["saturation"]);
      if (doc.containsKey("hmirror"))   s->set_hmirror(s, doc["hmirror"]);
      if (doc.containsKey("vflip"))     s->set_vflip(s, doc["vflip"]);
      if (doc.containsKey("quality"))   s->set_quality(s, doc["quality"]);

      if (doc.containsKey("framesize")) {
          framesize_t fs = (framesize_t)doc["framesize"].as<int>();
          s->set_framesize(s, fs);
      }

      // Persist to NVS
      camPrefs.begin("cam", false);
      for (JsonPair kv : doc.as<JsonObject>()) {
          // only ints are used in this structure
          camPrefs.putInt(kv.key().c_str(), kv.value().as<int>());
      }
      camPrefs.end();

      web.send(200, "application/json", "{\"ok\":true}");
  });

  // Apply camera defaults
  web.on("/api/cam_defaults", HTTP_ANY, []() {
      api_log("API /api/cam_defaults called");
      sensor_t* s = esp_camera_sensor_get();
      s->set_brightness(s, 0);
      s->set_contrast(s, 0);
      s->set_saturation(s, 0);
      s->set_aec2(s, 1);
      s->set_awb_gain(s, 1);
      web.send(200, "application/json", "{\"ok\":true}");
  });

  // Generic camera parameter setter
  web.on("/api/set_cam_param", HTTP_ANY, []() {
      if (!web.hasArg("param") || !web.hasArg("value")) {
          web.send(400, "application/json", "{\"error\":\"missing param or value\"}");
          return;
      }

      String p = web.arg("param");
      int v = web.arg("value").toInt();

      sensor_t* s = esp_camera_sensor_get();
      if (!s) {
          web.send(500, "application/json", "{\"error\":\"no sensor\"}");
          return;
      }

      bool ok = true;

      if (p == "brightness") s->set_brightness(s, v);
      else if (p == "contrast") s->set_contrast(s, v);
      else if (p == "saturation") s->set_saturation(s, v);
      else if (p == "sharpness") s->set_sharpness(s, v);
      else if (p == "denoise") s->set_denoise(s, v);
      else if (p == "ae_level") s->set_ae_level(s, v);
      else if (p == "agc_gain") s->set_agc_gain(s, v);
      else if (p == "aec2") s->set_aec2(s, v);
      else if (p == "aec_value") s->set_aec_value(s, v);
      else if (p == "awb") s->set_whitebal(s, v);
      else if (p == "awb_gain") s->set_awb_gain(s, v);
      else if (p == "hmirror") s->set_hmirror(s, v);
      else if (p == "vflip") s->set_vflip(s, v);
      else if (p == "quality") s->set_quality(s, v);
      else if (p == "gainceiling") s->set_gainceiling(s, (gainceiling_t)v);
      else if (p == "framesize") s->set_framesize(s, (framesize_t)v);
      else ok = false;

      if (!ok) {
          web.send(400, "application/json", "{\"error\":\"unknown param\"}");
          return;
      }

      // Persist single setting
      camPrefs.begin("cam", false);
      camPrefs.putInt(p.c_str(), v);
      camPrefs.end();

      web.send(200, "application/json", "{\"ok\":true}");
  });

  // Timezone + clock APIs
  web.on("/api/set_tz", HTTP_ANY, []() {
      if (!web.hasArg("tz")) {
          web.send(400, "application/json", "{\"error\":\"missing tz\"}");
          return;
      }
      String tz_arg = web.arg("tz");
      prefs.putString("timezone", tz_arg);
      setenv("TZ", tz_arg.c_str(), 1);
      tzset();

      web.send(200, "application/json", "{\"ok\":true}");
  });

  web.on("/api/sync_clock", HTTP_ANY, []() {
      if (!web.hasArg("epoch")) {
          web.send(400, "application/json", "{\"error\":\"missing epoch\"}");
          return;
      }

      // Apply timezone BEFORE setting the clock, so struct tm always aligns
      if (web.hasArg("tz")) {
          String tz_arg = web.arg("tz");
          prefs.putString("timezone", tz_arg);
          setenv("TZ", tz_arg.c_str(), 1);
          tzset();
          Serial.printf("TZ updated via /api/sync_clock: %s\n", tz_arg.c_str());
      }

      time_t t = (time_t)web.arg("epoch").toInt();

      struct timeval tv;
      tv.tv_sec  = t;
      tv.tv_usec = 0;

      // Manual epoch set
      settimeofday(&tv, nullptr);

      Serial.printf("Clock synced manually via /api/sync_clock: %ld\n", (long)t);

      web.send(200, "application/json", "{\"ok\":true}");
  });

  // Return stored settings (timezone, temp format, stream state)
  web.on("/api/get_settings", HTTP_GET, []() {
      String tz = prefs.getString("timezone", "UTC0");

      char json[128];
      snprintf(json, sizeof(json),
               "{\"timezone\":\"%s\",\"stream_on\":%s,\"temp_format\":\"%c\"}",
               tz.c_str(),
               stream_default_on ? "true" : "false",
               show_fahrenheit ? 'F' : 'C');

      web.send(200, "application/json", json);
  });

  web.on("/api/toggle_stream_default", HTTP_ANY, []() {
      stream_default_on = !stream_default_on;
      prefs.putBool("stream_default", stream_default_on);

      web.send(200, "application/json",
               stream_default_on
               ? "{\"ok\":true,\"stream_on\":true}"
               : "{\"ok\":true,\"stream_on\":false}");
  });

  web.on("/toggle_temp", HTTP_GET, []() {
      show_fahrenheit = !show_fahrenheit;
      prefs.putBool("tempF", show_fahrenheit);

      // Lightweight JSON response so fetch() is happy
      web.send(200, "application/json",
               show_fahrenheit
               ? "{\"ok\":true,\"temp_format\":\"F\"}"
               : "{\"ok\":true,\"temp_format\":\"C\"}");
  });

  web.on("/sync", HTTP_GET, handle_sync);

  // UI control (redirects)
  web.on("/start", HTTP_GET, []() {
    set_stream(true);
    redirect_home();
  });
  web.on("/stop", HTTP_GET, []() {
    set_stream(false);
    redirect_home();
  });
  web.on("/flash", HTTP_GET, []() {
    int val = 0;
    if (web.hasArg("val")) {
      val = constrain(web.arg("val").toInt(), 0, 255);
    }
    set_flash((uint8_t)val);
    redirect_home();
  });

  // API control (JSON)
  web.on("/api/start", HTTP_ANY, handle_api_start);
  web.on("/api/stop", HTTP_ANY, handle_api_stop);
  web.on("/api/flash", HTTP_ANY, handle_api_flash);
  
  // Favicon
  web.on("/favicon.ico", HTTP_GET, []() {
      web.sendHeader("Content-Type", "image/png");
      web.send(200, "image/png", FAVICON_BASE64);
  });

  web.onNotFound([]() {
    web.send(404, "text/plain", "Not found");
  });

  web.begin();
  Serial.println("Web server started on port 80.");

  // Start stream according to stored default once everything is ready
  set_stream(stream_default_on);

  last_telem_ms        = millis();
  last_mqtt_attempt_ms = millis();
}

// =============================================================
//  LOOP
// =============================================================
void loop() {
  // OTA
  if (ota_enabled) {
      ArduinoOTA.handle();
  }

  // Web
  web.handleClient();

  // MQTT: non-blocking, rate-limited reconnect
  if (mqtt.connected()) {
    mqtt.loop();
  } else {
    uint32_t now = millis();
    if (now - last_mqtt_attempt_ms > MQTT_RETRY_INTERVAL_MS) {
      last_mqtt_attempt_ms = now;
      mqtt_connect_once();
    }
  }

  // Telemetry
  if (millis() - last_telem_ms >= TELEMETRY_INTERVAL_MS) {
    publish_telemetry();
    last_telem_ms = millis();
  }

  // Flash auto-off
  if (led_active && (millis() - led_on_ms > FLASH_AUTO_OFF_MS)) {
    set_flash(0);
    log_line("Flash auto-off after timeout", true);
  }

  // RTSP handling: send frames when server is ready and streaming is enabled
  if (stream_on && rtspServer.readyToSendFrame()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      sensor_t* s = esp_camera_sensor_get();
      int quality = s ? s->status.quality : 10;  // fall back to something sane

      // Use actual frame dimensions from the sensor
      rtspServer.sendRTSPFrame(fb->buf, fb->len, quality, fb->width, fb->height);

      esp_camera_fb_return(fb);
    }
  }
}

