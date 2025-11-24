#pragma once

// ---- Wi-Fi ----
#define WIFI_SSID           ""
#define WIFI_PASS           ""

// ---- Optional Static Network Configuration ----
// Leave as empty strings ("") to use DHCP defaults.
#define STA_IP              ""          // e.g. "192.168.2.230"
#define STA_GATEWAY         ""          // e.g. "192.168.2.1"
#define STA_SUBNET          ""
#define STA_DNS             ""          // optional, defaults to gateway if blank

// ---- DEVICE NAME ----
#define DEVICE_NAME         "esp32_camera"  // used in UI and OTA hostname

// ---- MQTT ----
#define MQTT_SERVER         ""          // e.g. "192.168.2.230"
#define MQTT_PORT           1883        // default 1883
#define MQTT_USER           ""
#define MQTT_PASS           ""

// Client ID for MQTT; keep unique per device
#define MQTT_CLIENT_ID      DEVICE_NAME // Default is DEVICE_NAME

// Topics
#define MQTT_TOPIC_CMD      "/esp32cam/cmd"
#define MQTT_TOPIC_STATUS   "/esp32cam/status"
#define MQTT_TOPIC_TELEM    "/esp32cam/telemetry"
#define MQTT_TOPIC_VERBOSE  "/esp32cam/status_verbose"

// ---- RTSP ----
#define RTSP_PORT           8554
#define RTSP_USER           ""
#define RTSP_PASSWD         ""

// ---- OTA (ArduinoOTA) ----
#define OTA_HOSTNAME        DEVICE_NAME // Default is DEVICE_NAME
#define OTA_PASSWORD        ""
#define OTA_PORT            ""          // Default port is 3232
#define OTA_IP              ""
