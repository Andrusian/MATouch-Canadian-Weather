#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// ----------------------------------------------------------------
// WiFiProvisioner
//
// Phase 1:  On first boot (or if stored credentials fail), launches
//           a captive-portal AP called "MaTouch-Setup" so the user
//           can enter WiFi credentials via a phone/laptop browser.
//           Credentials are saved to NVS via Preferences and reused
//           on subsequent boots without re-entering the portal.
//
// Phase 1b: After connection, syncs time via NTP.
// ----------------------------------------------------------------

class WiFiProvisioner {
public:
    // Call once from setup(). Blocks until connected or portal times out.
    // Returns true if connected, false if portal timed out with no creds.
    bool begin(std::function<void(const String&)> statusCallback = nullptr) {
        _statusCb = statusCallback;

        status("Starting WiFi...");

        WiFiManager wm;
        wm.setConfigPortalTimeout(180);  // 3 min portal timeout
        wm.setSaveConnectTimeout(30);

        // Callback fires when portal is opened
        wm.setAPCallback([this](WiFiManager* wm) {
            status("AP portal open: connect to 'MaTouch-Setup'");
        });

        // Callback fires when new credentials are saved
        wm.setSaveParamsCallback([this]() {
            status("Credentials saved.");
        });

        bool connected = wm.autoConnect("MaTouch-Setup", /* password */ nullptr);

        if (!connected) {
            status("WiFi: portal timed out, no connection.");
            return false;
        }

        String ip = WiFi.localIP().toString();
        status("WiFi connected. IP: " + ip);

        _syncNTP();
        return true;
    }

    // Returns true if currently connected
    bool isConnected() const {
        return WiFi.status() == WL_CONNECTED;
    }

    // Call from loop() to handle reconnection
    void maintain() {
        if (!isConnected()) {
            status("WiFi lost, reconnecting...");
            WiFi.reconnect();
        }
    }

    // NTP accessors
    String getTimeString() {
        if (!_ntpReady) return "--:--:--";
        _ntp->update();
        return _ntp->getFormattedTime();
    }

    unsigned long getEpoch() {
        if (!_ntpReady) return 0;
        return _ntp->getEpochTime();
    }

private:
    std::function<void(const String&)> _statusCb;
    WiFiUDP     _udp;
    NTPClient*  _ntp     = nullptr;
    bool        _ntpReady = false;

    void status(const String& msg) {
        Serial.println(msg);
        if (_statusCb) _statusCb(msg);
    }

    void _syncNTP() {
        status("Syncing NTP...");
        // Use pool.ntp.org; adjust UTC offset (seconds) for your timezone.
        // EST = -18000, EDT = -14400. You can make this configurable later.
        const long utcOffsetSeconds = -18000;  // EST (Ontario)
        _ntp = new NTPClient(_udp, "pool.ntp.org", utcOffsetSeconds, 60000);
        _ntp->begin();
        int attempts = 0;
        while (!_ntp->update() && attempts++ < 10) {
            delay(500);
        }
        if (_ntp->isTimeSet()) {
            _ntpReady = true;
            status("Time: " + _ntp->getFormattedTime());
        } else {
            status("NTP sync failed (will retry).");
        }
    }
};