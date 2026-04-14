#pragma once
#include <Arduino.h>
#include <ArduinoOTA.h>

// ----------------------------------------------------------------
// OTAManager  (stub - Phase 1 placeholder)
//
// Enables ArduinoOTA so you can push firmware over WiFi from
// PlatformIO ("Upload" with the ota environment, or pio run -t upload
// --upload-port <ip>).
//
// Expand later with: version checking, progress display on screen,
// authenticated updates, or HTTP OTA from a server.
// ----------------------------------------------------------------

class OTAManager {
public:
    void begin(const char* hostname = "matouch") {
        ArduinoOTA.setHostname(hostname);

        ArduinoOTA.onStart([]() {
            Serial.println("OTA: starting update...");
        });
        ArduinoOTA.onEnd([]() {
            Serial.println("OTA: done, rebooting.");
        });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("OTA: %u%%\r", (progress * 100) / total);
        });
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("OTA error [%u]\n", error);
        });

        ArduinoOTA.begin();
        Serial.println("OTA ready.");
    }

    // Call from loop()
    void handle() {
        ArduinoOTA.handle();
    }
};
