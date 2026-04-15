#include <Arduino.h>

#define DEBUG   // Comment out for production
#include "LovyanGFX_config.h"
#include "WiFiProvisioner.h"
#include "OTAManager.h"
#include "TileCalc.h"
#include "RadarMap.h"
#include "TouchDimmer.h"

// ----------------------------------------------------------------
// MaTouch Weather Radar
// Phase 1a: OSM basemap fetch + stitch + display        (this build)
// Phase 1b: OWM radar overlay at 50% alpha              (this build)
// Phase 1ccd : Lightning strike circles                    (next)
// Phase 2:  Centre-tap zoom cycling                     (this build)
// ----------------------------------------------------------------

LGFX            display;
WiFiProvisioner wifi;
OTAManager      ota;
RadarMap*       radar  = nullptr;
TouchDimmer*    dimmer = nullptr;

// --- Boot status display ---
static int statusY = 10;
void showStatus(const String& msg) {
    Serial.println(msg);
    display.setTextSize(1);
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.setCursor(8, statusY);
    display.print(msg);
    statusY += 14;
    if (statusY > display.height() - 20) {
        display.fillScreen(TFT_BLACK);
        statusY = 10;
    }
}

void setup() {
    delay(4000);
    Serial.begin(115200);

    display.init();
    display.setBrightness(TouchDimmer::BRIGHTNESS_FULL);
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextSize(2);
    display.setCursor(8, 8);
    display.println("MaTouch Canadian Radar");
    statusY = 32;
    showStatus("Display OK");

    bool wifiOk = wifi.begin([](const String& msg) {
        showStatus(msg);
    });

    if (wifiOk) {
        ota.begin("matouch-radar");
        showStatus("Ready.");
    } else {
        showStatus("No WiFi - offline mode.");
    }

    Serial.printf("\nFree heap:  %u\n", ESP.getFreeHeap());
    Serial.printf("Free PSRAM: %u\n", ESP.getFreePsram());
    Serial.printf("PSRAM size: %u\n", ESP.getPsramSize());

    dimmer = new TouchDimmer(
        display.width(),
        display.height(),
        [](uint8_t b) { display.setBrightness(b); }
    );

    if (wifiOk) {
        showStatus("Allocating radar...");
        radar = new RadarMap(&display);
        if (!radar->begin()) {
            showStatus("RadarMap init failed!");
            delete radar;
            radar = nullptr;
        }
        // RadarMap::begin() fetches basemap and triggers immediate
        // radar overlay — display is live by the time begin() returns.
    } else {
        showStatus("No radar without WiFi.");
    }
}

void loop() {
    ota.handle();
    wifi.maintain();
    dimmer->update();

    // Inject current time into radar for timestamp overlay
    if (radar) {
        String t = wifi.getTimeString();
        radar->setTimeString(t.c_str());
    }

    // --- Touch handling ---
    uint16_t tx, ty;
    if (display.getTouch(&tx, &ty)) {
        // Serial.printf("[touch] x=%3d  y=%3d\n", tx, ty);

        bool consumed = dimmer->onTouch(tx, ty);

        if (!consumed && radar) {
            consumed = radar->onTouch(tx, ty);
        }
        // Future: other touch handlers here
    }

    // --- Radar refresh (RadarMap handles its own timer internally) ---
    if (radar) {
        radar->update(false);
    }
}