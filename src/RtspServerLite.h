#pragma once
#include <WiFiClient.h>
#include "CamStreamer.h"
#include "CRtspSession.h"

/**
 * Lightweight wrapper around Micro-RTSP’s CRtspSession.
 * Handles one client at a time.
 */
class RtspServerLite {
public:
    RtspServerLite(int port, u_short width = 640, u_short height = 480)
        : tcpServer(port), width(width), height(height) {}

    void begin() { tcpServer.begin(); }

    void handleConnections(bool stream_on) {
        WiFiClient client = tcpServer.available();
        if (!client) return;

        CamStreamer streamer(width, height);
        CRtspSession session(&client, &streamer);

        // main RTSP loop
        while (client.connected() && stream_on) {
            session.handleRequests(0);
            streamer.streamImage(millis());
            yield();
        }
    }

private:
    WiFiServer tcpServer;
    u_short width;
    u_short height;
};
