#pragma once
#include "LovyanGFX_config.h"
#include <HTTPClient.h>

class JpegFetcher {
public:
    JpegFetcher(LGFX* display) : _display(display) {}

    bool fetchAndDraw(const char* url, int x = 0, int y = 0,
                      int maxW = 480, int maxH = 320,
                      bool scale = true) {
        HTTPClient http;
        http.begin(url);
        http.setTimeout(10000);
        int code = http.GET();

        if (code != HTTP_CODE_OK) {
            Serial.printf("[JpegFetcher] HTTP GET failed: %d\n", code);
            http.end();
            return false;
        }

        int len = http.getSize();
        int bufSize = (len > 0) ? len : 150000;

        uint8_t* buf = (uint8_t*)heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
        if (!buf) {
            Serial.printf("[JpegFetcher] PSRAM alloc failed for %d bytes\n", bufSize);
            http.end();
            return false;
        }

        WiFiClient* stream = http.getStreamPtr();
        int bytesRead = 0;
        uint32_t deadline = millis() + 10000;

        while ((http.connected() || stream->available()) &&
               bytesRead < bufSize &&
               millis() < deadline) {
            int avail = stream->available();
            if (avail) {
                bytesRead += stream->readBytes(buf + bytesRead, avail);
            } else {
                delay(1);
            }
        }

        http.end();
        Serial.printf("[JpegFetcher] Fetched %d bytes\n", bytesRead);

        if (bytesRead > 0) {
            int imgW = 0, imgH = 0;
            for (int i = 0; i < bytesRead - 8; i++) {
                if (buf[i] == 0xFF &&
                   (buf[i+1] == 0xC0 || buf[i+1] == 0xC2)) {
                    imgH = (buf[i+5] << 8) | buf[i+6];
                    imgW = (buf[i+7] << 8) | buf[i+8];
                }
            }
            Serial.printf("[JpegFetcher] Image: %dx%d, target: %dx%d\n",
                          imgW, imgH, maxW, maxH);

            if (scale && imgW > 0 && (imgW > maxW || imgH > maxH)) {
                float sx = (float)maxW / imgW;
                float sy = (float)maxH / imgH;
                float s  = min(sx, sy);
                int drawW = (int)(imgW * s);
                int drawH = (int)(imgH * s);
                int offX  = x + (maxW - drawW) / 2;
                int offY  = y + (maxH - drawH) / 2;
                Serial.printf("[JpegFetcher] Scale: %.3f  output: %dx%d @ (%d,%d)\n",
                              s, drawW, drawH, offX, offY);
                _display->drawJpg(buf, bytesRead,
                                  offX, offY,
                                  drawW, drawH,
                                  0, 0,
                                  s, s);
            } else {
                _display->drawJpg(buf, bytesRead, x, y, maxW, maxH);
            }
        }

        heap_caps_free(buf);
        return bytesRead > 0;
    }

private:
    LGFX* _display;
};