#pragma once

#include "CStreamer.h"
#include "esp_camera.h"

extern bool stream_on;

// Simple streamer that pulls JPEG frames from esp_camera
// and hands them to the CStreamer RTP machinery.
class CamStreamer : public CStreamer
{
public:
    CamStreamer(u_short width, u_short height)
        : CStreamer(width, height),
          mFrameBuf(nullptr),
          mFrameSize(0)
    {}

    virtual ~CamStreamer() {
        if (mFrameBuf) {
            free(mFrameBuf);
            mFrameBuf = nullptr;
            mFrameSize = 0;
        }
    }

    // Called by CRtspSession via CStreamer::handleRequests()
    // to push a new frame out over RTP.
    void streamImage(uint32_t curMsec) override
    {
        // Respect global stream_on flag: don't send frames if disabled
        if (!stream_on) {
            return;
        }

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            return;
        }

        // Cache length before releasing the frame buffer
        size_t jpegLen = fb->len;

        // Ensure our local buffer is large enough
        if (jpegLen > mFrameSize) {
            uint8_t* newBuf = (uint8_t*)realloc(mFrameBuf, jpegLen);
            if (!newBuf) {
                // Allocation failed: drop this frame safely
                esp_camera_fb_return(fb);
                return;
            }
            mFrameBuf = newBuf;
            mFrameSize = jpegLen;
        }

        // Copy JPEG payload into our own buffer,
        // then immediately release the camera frame.
        memcpy(mFrameBuf, fb->buf, jpegLen);
        esp_camera_fb_return(fb);

        // Hand off to RTSP base class. It will do JPEG header parsing,
        // quant tables, and RTP packetization as needed.
        streamFrame((BufPtr)mFrameBuf, (uint32_t)jpegLen, curMsec);
    }

private:
    uint8_t* mFrameBuf;   // heap buffer holding last JPEG frame
    size_t   mFrameSize;  // size of allocated buffer
};

