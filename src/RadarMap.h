#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "LovyanGFX_config.h"
#include "TileCalc.h"

// ----------------------------------------------------------------
// RadarMap
//
// Phase 1a: Fetch OSM basemap tiles → stitch into PSRAM sprite →
//           crop and blit 480×320 to display. Basemap cached in
//           _basemap sprite; only re-fetched on zoom change or boot.
//
// Phase 1b: Fetch OWM precipitation tiles → draw over _composite
//           sprite (copy of basemap) at 50% alpha → push to display.
//           Refreshed every RADAR_REFRESH_MS.
//
// Phase 1c: Lightning strike circles drawn over composite.
//
// Phase 2:  Centre-tap cycles zoom 400km → 100km → 50km → 400km.
//           Zoom change invalidates basemap and triggers immediate
//           full refresh.
//
// PSRAM budget (16bpp sprites):
//   _basemap   480×320 × 2 = 307,200 bytes  (~300 KB)
//   _composite 480×320 × 2 = 307,200 bytes  (~300 KB)
//   Tile fetch buffer       ≤ 50,000 bytes  (~ 50 KB, reused)
//   Total                                   (~650 KB of ~8 MB)
// ----------------------------------------------------------------

// OWM API key — set before calling begin()
#define OWM_API_KEY  "ac8451a3eb4a93263fd0f9f73b52f417"

// Refresh interval for radar overlay (ms)
#define RADAR_REFRESH_MS  (5UL * 60UL * 1000UL)   // 5 minutes

// Centre points per zoom level
// Define DEBUG above this include to use test coordinates
#ifdef ALTERNATE
static const double CENTRE_LAT[3] = {
    40.75,    // z=7  400km  DEBUG: active storm area
    40.75,    // z=9  100km
    40.75     // z=10  50km
};
static const double CENTRE_LON[3] = {
    -73.926,   // z=7  400km
    -73.926,   // z=9  100km
    -73.926    // z=10  50km
};
#else
static const double CENTRE_LAT[3] = {
    42.9574481183419,      // z=7  400km
    42.9574481183419,      // z=9  100km
    42.81111151466509      // z=10  50km — weather-optimised SSW of St Thomas
};
static const double CENTRE_LON[3] = {
    -81.16835498354426,    // z=7  400km
    -81.16835498354426,    // z=9  100km
    -81.25584709039006     // z=10  50km
};
#endif

// Display dimensions (post-rotation)
static constexpr int DISP_W = 480;
static constexpr int DISP_H = 320;

// Centre-tap detection zone (pixels from display centre)
static constexpr int TAP_CENTRE_ZONE = 60;

// PNG tile fetch buffer size — 256×256 OSM tiles are typically 10-40 KB
static constexpr int TILE_BUF_SIZE = 50000;  // internal RAM — keep lean

// Alpha for radar overlay: 0=transparent, 255=opaque. 128 ≈ 50%.
static constexpr uint8_t RADAR_ALPHA = 128;

class RadarMap {
public:
    RadarMap(LGFX* display)
        : _display(display),
          _zoomIndex(0),
          _basemapValid(false),
          _lastRadarMs(0),
          _basemap(nullptr),
          _composite(nullptr),
          _tileBuf(nullptr)
    {}

    // ------------------------------------------------------------------
    // begin() — call once from setup() after WiFi is connected.
    // Allocates PSRAM sprites and tile buffer, then fetches basemap.
    // ------------------------------------------------------------------
    bool begin() {
        Serial.printf("[RadarMap] Free internal RAM: %u\n",
                      heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        Serial.printf("[RadarMap] Free PSRAM before alloc: %u\n",
                      ESP.getFreePsram());

        // Allocate basemap sprite in PSRAM
        _basemap = new lgfx::LGFX_Sprite(_display);
        _basemap->setPsram(true);
        if (!_basemap->createSprite(DISP_W, DISP_H)) {
            Serial.println("[RadarMap] ERROR: basemap sprite alloc failed");
            return false;
        }
        _basemap->fillScreen(TFT_BLACK);

        // Allocate composite sprite in PSRAM
        _composite = new lgfx::LGFX_Sprite(_display);
        _composite->setPsram(true);
        if (!_composite->createSprite(DISP_W, DISP_H)) {
            Serial.println("[RadarMap] ERROR: composite sprite alloc failed");
            return false;
        }
        _composite->fillScreen(TFT_BLACK);

        // Allocate reusable tile fetch buffer in PSRAM
        // Internal RAM required — LovyanGFX pngle decoder cannot safely
        // access PSRAM (OPI cache alignment faults on arbitrary offsets).
        _tileBuf = (uint8_t*)heap_caps_malloc(TILE_BUF_SIZE, MALLOC_CAP_INTERNAL);
        if (!_tileBuf) {
            // Fallback: try smaller buffer in internal RAM
            Serial.printf("[RadarMap] Internal RAM alloc failed,"
                          " free internal: %u\n",
                          heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        }
        if (!_tileBuf) {
            Serial.println("[RadarMap] ERROR: tile buffer alloc failed");
            return false;
        }

        Serial.printf("[RadarMap] Free PSRAM after alloc:  %u\n",
                      ESP.getFreePsram());

        // Fetch basemap for default zoom level
        _fetchBasemap();

        // Trigger immediate radar overlay fetch
        _lastRadarMs = millis() - RADAR_REFRESH_MS;

        return true;
    }

    // ------------------------------------------------------------------
    // update() — call every loop() iteration.
    // Handles radar refresh timer and pushes composite to display.
    // Skip while dimmed (preserves API quota).
    // ------------------------------------------------------------------
    void update(bool dimmed) {
        if (dimmed) return;

        uint32_t now = millis();
        if ((now - _lastRadarMs) >= RADAR_REFRESH_MS) {
            _lastRadarMs = now;
            _buildComposite();   // copy basemap + draw radar overlay
            _pushToDisplay();
        }
    }

    // ------------------------------------------------------------------
    // onTouch() — call from main touch handler (after TouchDimmer).
    // Returns true if the touch was consumed by RadarMap (zoom change).
    // ------------------------------------------------------------------
    bool onTouch(uint16_t tx, uint16_t ty) {
        // Centre zone: within TAP_CENTRE_ZONE px of display centre
        int dx = (int)tx - DISP_W / 2;
        int dy = (int)ty - DISP_H / 2;
        if (abs(dx) > TAP_CENTRE_ZONE || abs(dy) > TAP_CENTRE_ZONE)
            return false;

        // Cycle zoom level
        _zoomIndex = (_zoomIndex + 1) % ZOOM_LEVEL_COUNT;
        Serial.printf("[RadarMap] Zoom -> index %d  z=%d\n",
                      _zoomIndex, ZOOM_LEVELS[_zoomIndex]);

        // Invalidate basemap and fetch fresh for new zoom
        _basemapValid = false;
        _fetchBasemap();

        // Immediate radar refresh
        _lastRadarMs = millis() - RADAR_REFRESH_MS;

        return true;
    }

    // Current zoom level label for UI
    const char* zoomLabel() const {
        switch (_zoomIndex) {
            case 0: return "400km";
            case 1: return "100km";
            default: return " 50km";
        }
    }

private:
    LGFX*              _display;
    lgfx::LGFX_Sprite* _basemap;
    lgfx::LGFX_Sprite* _composite;
    uint8_t*           _tileBuf;

    int      _zoomIndex;
    bool     _basemapValid;
    uint32_t _lastRadarMs;

    // ------------------------------------------------------------------
    // _currentGrid() — TileGrid for the active zoom level
    // ------------------------------------------------------------------
    TileGrid _currentGrid() const {
        int z = ZOOM_LEVELS[_zoomIndex];
        return TileCalc::tileGrid(
            CENTRE_LAT[_zoomIndex],
            CENTRE_LON[_zoomIndex],
            z, DISP_W, DISP_H);
    }

    // ------------------------------------------------------------------
    // _fetchBasemap() — fetch OSM PNG tiles and stitch into _basemap.
    // Uses a temporary canvas sprite sized to the tile grid, draws each
    // tile into it, then crops DISP_W×DISP_H into _basemap.
    // ------------------------------------------------------------------
    void _fetchBasemap() {
        TileGrid g = _currentGrid();
        TileCalc::printGrid(g, zoomLabel());

        _basemap->fillScreen(TFT_DARKGREY);  // visible if tiles fail

        // Temporary canvas sized to full tile grid
        lgfx::LGFX_Sprite canvas(_display);
        canvas.setPsram(true);
        if (!canvas.createSprite(g.canvasW(), g.canvasH())) {
            Serial.printf("[RadarMap] Canvas alloc failed (%dx%d)\n",
                          g.canvasW(), g.canvasH());
            return;
        }
        canvas.fillScreen(TFT_DARKGREY);

        int totalTiles = g.colCount * g.rowCount;
        Serial.printf("[RadarMap] Fetching %d basemap tiles...\n", totalTiles);
        uint32_t t0 = millis();

        int serial = 0;
        int fetched = 0;
        for (int row = 0; row < g.rowCount; row++) {
            for (int col = 0; col < g.colCount; col++) {
                // Destination rect in stitched canvas
                int cx = col * 256;
                int cy = row * 256;

                // Destination rect on display after crop —
                // skip tiles entirely outside the display window
                int dx = cx - g.cropX;
                int dy = cy - g.cropY;
                if (dx >= DISP_W || dy >= DISP_H ||
                    dx + 256 <= 0 || dy + 256 <= 0) {
                    Serial.printf("[RadarMap]   tile[%d,%d] off-screen, skip\n",
                                  col, row);
                    serial++;
                    continue;
                }

                int tx = g.tileX0 + col;
                int ty = g.tileY0 + row;

                char url[128];
                TileCalc::osmUrl(url, sizeof(url), g.z, tx, ty, serial++);
                Serial.printf("[RadarMap]   OSM %s\n", url);

                int bytes = _fetchPng(url);
                if (bytes > 0) {
                    int pngLen = bytes - _pngOffset;
                    canvas.drawPng(_tileBuf + _pngOffset, (uint32_t)pngLen,
                                   cx, cy, 256, 256);
                    fetched++;
                } else {
                    // Fill failed tile with a visible placeholder
                    canvas.fillRect(cx+1, cy+1, 254, 254, TFT_DARKGREY);
                    canvas.drawRect(cx, cy, 256, 256, TFT_RED);
                }

                delay(50);  // polite to OSM tile servers
            }
        }

        uint32_t elapsed = millis() - t0;
        Serial.printf("[RadarMap] Basemap: %d/%d tiles in %lu ms (%.1f s)\n",
                      fetched, totalTiles, elapsed, elapsed / 1000.0f);

        // Crop canvas → _basemap sprite
        // pushSprite(dst, x, y) pushes 'this' to dst at (x,y) —
        // we want the crop window's top-left to land at (0,0) on _basemap,
        // so we pass negative offsets.
        canvas.pushSprite(_basemap, -g.cropX, -g.cropY);
        canvas.deleteSprite();

        _drawScaleBar(_basemap, ZOOM_LEVELS[_zoomIndex],
                      CENTRE_LAT[_zoomIndex]);
        _drawZoomLabel(_basemap);

        _basemapValid = true;
        Serial.println("[RadarMap] Basemap ready.");
    }

    // ------------------------------------------------------------------
    // _buildComposite() — copy basemap → composite, then draw radar tiles
    // over it at RADAR_ALPHA transparency.
    //
    // Alpha strategy: drawPng() has no alpha parameter in LovyanGFX 1.1.x.
    // Instead we decode each radar tile into a temporary 256×256 sprite,
    // then pushSprite() onto _composite with alpha=RADAR_ALPHA (0-255).
    // The OWM precipitation PNGs are RGBA — transparent where no precip.
    // drawPng() into the tile sprite respects that native PNG alpha, so
    // clear areas stay transparent in the sprite's pixel buffer.
    // pushSprite() with our additional alpha dims the whole overlay uniformly.
    // ------------------------------------------------------------------
    void _buildComposite() {
        if (!_basemapValid) {
            Serial.println("[RadarMap] Basemap not valid, fetching...");
            _fetchBasemap();
        }

        // Start composite from fresh copy of basemap
        _basemap->pushSprite(_composite, 0, 0);

        TileGrid g = _currentGrid();
        Serial.printf("[RadarMap] Fetching radar tiles (z=%d)...\n", g.z);

        int serial = 0;
        int fetched = 0;
        for (int row = 0; row < g.rowCount; row++) {
            for (int col = 0; col < g.colCount; col++) {
                int cx = col * 256 - g.cropX;
                int cy = row * 256 - g.cropY;

                if (cx >= DISP_W || cy >= DISP_H ||
                    cx + 256 <= 0 || cy + 256 <= 0) {
                    serial++;
                    continue;
                }

                int tx = g.tileX0 + col;
                int ty = g.tileY0 + row;

                char url[256];
                TileCalc::owmUrl(url, sizeof(url),
                                 "precipitation_new",
                                 g.z, tx, ty, OWM_API_KEY);
                Serial.printf("[RadarMap]   radar %d/%d/%d\n", g.z, tx, ty);

                int bytes = _fetchPng(url);
                if (bytes > 0) {
                    int pngLen = bytes - _pngOffset;
                    const uint8_t* pngData = _tileBuf + _pngOffset;

#ifdef DEBUG
                    if (fetched == 0) {
                        _sampleTilePalette(pngData, pngLen);
                    }
#endif
                    // Decode radar tile into a temporary sprite, remap its
                    // colours, then copy onto composite. This way we only
                    // touch actual radar pixels — basemap is never modified.
                    lgfx::LGFX_Sprite tileSpr(_display);
                    tileSpr.setPsram(true);
                    if (tileSpr.createSprite(256, 256)) {
                        tileSpr.fillScreen(0x0000);
                        tileSpr.drawPng(pngData, (uint32_t)pngLen,
                                        0, 0, 256, 256);
                        _remapTileColours(&tileSpr);
                        // Blend remapped tile onto composite at RADAR_ALPHA
                        // opacity. Black pixels (transparent/suppressed) are
                        // skipped entirely — basemap shows through unchanged.
                        for (int ty2 = 0; ty2 < 256; ty2++) {
                            int dy = cy + ty2;
                            if (dy < 0 || dy >= DISP_H) continue;
                            for (int tx2 = 0; tx2 < 256; tx2++) {
                                int dx = cx + tx2;
                                if (dx < 0 || dx >= DISP_W) continue;
                                uint16_t rpx = tileSpr.readPixel(tx2, ty2);
                                if (rpx == 0x0000) continue;  // transparent

                                // Linear blend: out = radar*a + base*(1-a)
                                uint16_t bpx = _basemap->readPixel(dx, dy);
                                uint8_t rr = ((rpx>>11)&0x1F)<<3;
                                uint8_t rg = ((rpx>> 5)&0x3F)<<2;
                                uint8_t rb = ( rpx     &0x1F)<<3;
                                uint8_t br = ((bpx>>11)&0x1F)<<3;
                                uint8_t bg = ((bpx>> 5)&0x3F)<<2;
                                uint8_t bb = ( bpx     &0x1F)<<3;
                                uint8_t or2 = (rr*RADAR_ALPHA + br*(255-RADAR_ALPHA))>>8;
                                uint8_t og  = (rg*RADAR_ALPHA + bg*(255-RADAR_ALPHA))>>8;
                                uint8_t ob  = (rb*RADAR_ALPHA + bb*(255-RADAR_ALPHA))>>8;
                                uint16_t out = ((or2>>3)<<11)|((og>>2)<<5)|(ob>>3);
                                _composite->drawPixel(dx, dy, out);
                            }
                        }
                        tileSpr.deleteSprite();
                        fetched++;
                    } else {
                        Serial.println("[RadarMap] tile sprite alloc failed");
                    }
                }
                serial++;
                delay(50);
            }
        }

        Serial.printf("[RadarMap] Radar: %d tiles fetched\n", fetched);

        // Redraw UI overlays on top of radar
        _drawScaleBar(_composite, ZOOM_LEVELS[_zoomIndex],
                      CENTRE_LAT[_zoomIndex]);
        _drawZoomLabel(_composite);
        _drawTimestamp(_composite);
    }

    // ------------------------------------------------------------------
    // _pushToDisplay() — blit composite sprite to physical display
    // ------------------------------------------------------------------
    void _pushToDisplay() {
        _composite->pushSprite(0, 0);
    }

    // ------------------------------------------------------------------
    // _fetchPng() — fetch a PNG tile URL into _tileBuf.
    // Returns bytes read, or 0 on failure.
    // Handles both http:// and https:// URLs.
    // ------------------------------------------------------------------
    int _fetchPng(const char* url) {
        HTTPClient http;

        // Fresh WiFiClientSecure per call -- reusing a static instance across
        // different hosts (OSM then OWM) leaves TLS state corrupted.
        WiFiClientSecure secureClient;

        bool isHttps = (strncmp(url, "https://", 8) == 0);
        if (isHttps) {
            secureClient.setInsecure();
            http.begin(secureClient, url);
        } else {
            http.begin(url);
        }

        http.setTimeout(15000);
        http.setUserAgent("MaTouch-RadarMap/1.0");

        int code = http.GET();
        int len  = http.getSize();
        Serial.printf("[RadarMap] HTTP %d  len=%d\n", code, len);
        if (code != HTTP_CODE_OK) {
            http.end();
            return 0;
        }

        // OWM uses chunked transfer (len=-1); OSM sends Content-Length.
        // Cap reads at TILE_BUF_SIZE either way.
        int maxRead = (len > 0) ? min(len, TILE_BUF_SIZE) : TILE_BUF_SIZE;
        if (len > TILE_BUF_SIZE) {
            Serial.printf("[RadarMap] Tile too large: %d bytes\n", len);
            http.end();
            return 0;
        }

        WiFiClient* stream = http.getStreamPtr();
        stream->setTimeout(10000);

        int bytesRead = 0;

        if (len > 0) {
            // Content-Length known — simple read, no chunking to strip
            bytesRead = (int)stream->readBytes(_tileBuf, min(len, maxRead));
        } else {
            // Chunked transfer — must strip "SIZE\r\n...DATA...\r\n" framing.
            // Each chunk: hex-size CRLF, data bytes, CRLF.
            // Final chunk: "0\r\n\r\n"
            uint32_t deadline = millis() + 15000;
            while (bytesRead < maxRead && millis() < deadline) {
                // Read chunk size line (hex digits + CRLF)
                char sizeLine[16] = {0};
                int si = 0;
                uint32_t lineDeadline = millis() + 3000;
                while (millis() < lineDeadline && si < 15) {
                    if (stream->available()) {
                        char c = stream->read();
                        if (c == '\n') break;   // end of size line
                        if (c != '\r') sizeLine[si++] = c;
                    } else delay(1);
                }
                sizeLine[si] = 0;

                int chunkSize = (int)strtol(sizeLine, nullptr, 16);
                if (chunkSize == 0) break;  // final chunk

                // Read exactly chunkSize bytes of payload
                int got = (int)stream->readBytes(
                    _tileBuf + bytesRead,
                    min(chunkSize, maxRead - bytesRead));
                bytesRead += got;

                // Consume trailing CRLF after chunk data
                uint32_t crDeadline = millis() + 1000;
                int crCount = 0;
                while (crCount < 2 && millis() < crDeadline) {
                    if (stream->available()) {
                        stream->read();
                        crCount++;
                    } else delay(1);
                }
            }
        }

        http.end();

        if (bytesRead == 0) {
            Serial.printf("[RadarMap] No data received\n");
            return 0;
        }
        Serial.printf("[RadarMap] Read %d bytes\n", bytesRead);

        // Locate the PNG signature \x89PNG — in chunked responses a
        // chunk-size header may precede the actual PNG data.
        // Scan up to 16 bytes in to find it.
        int pngOffset = -1;
        for (int i = 0; i <= min(bytesRead - 4, 16); i++) {
            if (_tileBuf[i]   == 0x89 && _tileBuf[i+1] == 'P' &&
                _tileBuf[i+2] == 'N'  && _tileBuf[i+3] == 'G') {
                pngOffset = i;
                break;
            }
        }

        if (pngOffset < 0) {
            Serial.printf("[RadarMap] No PNG signature found (%d bytes). "
                          "First bytes:\n", bytesRead);
            int show = min(bytesRead, 64);
            for (int i = 0; i < show; i++) {
                char c = (char)_tileBuf[i];
                Serial.print((c >= 32 && c < 127) ? c : '.');
            }
            Serial.println();
            return 0;
        }

        if (pngOffset > 0) {
            Serial.printf("[RadarMap] PNG at offset %d\n", pngOffset);
        }
        // Store offset so callers can pass _tileBuf+_pngOffset to drawPng
        _pngOffset = pngOffset;
        return bytesRead;
    }

    // ------------------------------------------------------------------
    // UI overlays
    // ------------------------------------------------------------------

    // Scale bar — bottom-left corner
    void _drawScaleBar(lgfx::LGFX_Sprite* spr, int z, double lat) {
        float kpp = TileCalc::kmPerPixel(lat, z);

        // Pick a round-number bar width in km
        float targets[] = { 200, 100, 50, 25, 10, 5 };
        float barKm = targets[0];
        for (float t : targets) {
            if (t / kpp < 120) { barKm = t; break; }
        }
        int barPx = (int)(barKm / kpp);

        int x0 = 10, y0 = DISP_H - 18;
        int x1 = x0 + barPx;

        // Shadow for legibility over any map colour
        spr->drawFastHLine(x0+1, y0+1,   barPx, TFT_BLACK);
        spr->drawFastVLine(x0+1, y0+1,   6,     TFT_BLACK);
        spr->drawFastVLine(x1+1, y0+1,   6,     TFT_BLACK);
        // Bar
        spr->drawFastHLine(x0,   y0,     barPx, TFT_WHITE);
        spr->drawFastVLine(x0,   y0,     6,     TFT_WHITE);
        spr->drawFastVLine(x1,   y0,     6,     TFT_WHITE);

        char label[16];
        if (barKm >= 100) snprintf(label, sizeof(label), "%d km", (int)barKm);
        else              snprintf(label, sizeof(label), "%.0f km", barKm);

        spr->setTextSize(1);
        spr->setTextColor(TFT_BLACK, TFT_TRANSPARENT);
        spr->setCursor(x0 + barPx/2 - 12 + 1, y0 - 10 + 1);
        spr->print(label);
        spr->setTextColor(TFT_WHITE, TFT_TRANSPARENT);
        spr->setCursor(x0 + barPx/2 - 12, y0 - 10);
        spr->print(label);
    }

    // Zoom range label — top-left
    void _drawZoomLabel(lgfx::LGFX_Sprite* spr) {
        spr->setTextSize(1);
        spr->setTextColor(TFT_BLACK, TFT_TRANSPARENT);
        spr->setCursor(9, 9);
        spr->print(zoomLabel());
        spr->setTextColor(TFT_WHITE, TFT_TRANSPARENT);
        spr->setCursor(8, 8);
        spr->print(zoomLabel());
    }

    // Timestamp — top-right (epoch from NTP via WiFiProvisioner is
    // not directly accessible here; use millis-based uptime for now,
    // main.cpp can inject a time string via setTimeString())
    void _drawTimestamp(lgfx::LGFX_Sprite* spr) {
        if (_timeStr[0] == '\0') return;
        spr->setTextSize(1);
        spr->setTextColor(TFT_BLACK, TFT_TRANSPARENT);
        spr->setCursor(DISP_W - 51, 9);
        spr->print(_timeStr);
        spr->setTextColor(TFT_WHITE, TFT_TRANSPARENT);
        spr->setCursor(DISP_W - 52, 8);
        spr->print(_timeStr);
    }

    // ------------------------------------------------------------------
    // _remapRadarColours() — scan composite sprite pixels, compute
    // intensity score from RGB565, remap to conventional radar palette.
    // Pixels that score below DRIZZLE_THRESHOLD are cleared (suppressed).
    // Pixels matching basemap colours (not from radar layer) are left alone
    // by checking that the pixel isn't pure black (transparent OWM areas
    // remain black after drawPng so we skip those too).
    // ------------------------------------------------------------------
    // Remap a 256x256 radar tile sprite in-place.
    // Score = R*3 + G*2 + B on RGB565 pixel values.
    // Thresholds calibrated from observed OWM precipitation_new palette.
    // Adjust SUPPRESS_BELOW to taste for drizzle sensitivity.
    void _remapTileColours(lgfx::LGFX_Sprite* spr) {
        // Thresholds — all fit within observed score range 0-248.
        // Increase SUPPRESS_BELOW to suppress more drizzle.
        static constexpr int SUPPRESS_BELOW = 50;   // suppress drizzle
        static constexpr int THRESH_LIGHT  = 100;  // light green
        static constexpr int THRESH_MEDIUM   = 130;  // green
        static constexpr int THRESH_HEAVY  = 180;  // yellow
        static constexpr int THRESH_VERYHEAVY  = 210;  // orange
        static constexpr int THRESH_STUPIDHEAVY  = 210;  // orange
        static constexpr int THRESH_CELL  = 230;  // red
        // above that purple

        // Output colours RGB565 https://rgbcolorpicker.com/565
        static constexpr uint16_t COL_LGREEN = 0x7f6f;  
        static constexpr uint16_t COL_GREEN  = 0x04c0;  
        static constexpr uint16_t COL_YELLOW = 0xFFE0; 
        static constexpr uint16_t COL_ORANGE = 0xFC40; 
        static constexpr uint16_t COL_RED    = 0xF800; 
        static constexpr uint16_t COL_PURPLE    = 0x0935 ; 

        int remapped = 0, suppressed = 0;

        for (int y = 0; y < 256; y++) {
            for (int x = 0; x < 256; x++) {
                uint16_t px = spr->readPixel(x, y);
                if (px == 0x0000) continue;  // transparent — leave black

                uint8_t r = ((px >> 11) & 0x1F) << 3;
                uint8_t g = ((px >>  5) & 0x3F) << 2;
                uint8_t b = ( px        & 0x1F) << 3;
                int score = r * 3 + g * 2 + b;

                if (score < SUPPRESS_BELOW) {
                    spr->drawPixel(x, y, 0x0000);  // suppress — goes black
                    suppressed++;
                    continue;
                }

                uint16_t newCol;
                if      (score < THRESH_LIGHT) newCol = COL_LGREEN;
                else if (score < THRESH_MEDIUM)  newCol = COL_GREEN;
                else if (score < THRESH_HEAVY) newCol = COL_YELLOW;
                else if (score < THRESH_VERYHEAVY) newCol = COL_ORANGE;
                else if (score < THRESH_STUPIDHEAVY) newCol = COL_ORANGE;
                else if (score < THRESH_CELL) newCol = COL_RED;                    // storm cells
                else     newCol = COL_PURPLE;                                      // possible tornatoes

                spr->drawPixel(x, y, newCol);
                remapped++;
            }
        }
        Serial.printf("[RadarMap]   remap: %d px, %d suppressed\n",
                      remapped, suppressed);
    }

#ifdef DEBUG
    // ------------------------------------------------------------------
    // _sampleTilePalette() — decode a radar tile into a temp sprite,
    // scan all pixels, and print a histogram of unique colours to Serial.
    // Run once on the first fetched tile to identify OWM's palette.
    // ------------------------------------------------------------------
    void _sampleTilePalette(const uint8_t* pngData, int pngLen) {
        Serial.println("[Palette] Sampling first radar tile...");

        lgfx::LGFX_Sprite tmp(_display);
        tmp.setPsram(true);
        if (!tmp.createSprite(256, 256)) {
            Serial.println("[Palette] Sprite alloc failed");
            return;
        }
        tmp.fillScreen(0x0000);  // black = no data
        tmp.drawPng(pngData, (uint32_t)pngLen, 0, 0, 256, 256);

        // Count occurrences of each unique 16-bit colour (RGB565).
        // Use a simple array of (colour, count) pairs — keep top 32.
        struct Entry { uint16_t colour; int count; };
        static Entry hist[64];
        int histCount = 0;
        int transparent = 0;  // pixels that are pure black (were transparent)
        int total = 0;

        for (int y = 0; y < 256; y++) {
            for (int x = 0; x < 256; x++) {
                uint16_t px = tmp.readPixel(x, y);
                total++;

                // Pure black = transparent/no-precip in our sprite
                if (px == 0x0000) { transparent++; continue; }

                // Find in histogram
                bool found = false;
                for (int i = 0; i < histCount; i++) {
                    if (hist[i].colour == px) {
                        hist[i].count++;
                        found = true;
                        break;
                    }
                }
                if (!found && histCount < 64) {
                    hist[histCount++] = {px, 1};
                }
            }
        }

        tmp.deleteSprite();

        // Sort by count descending (simple bubble sort — one-time diagnostic)
        for (int i = 0; i < histCount - 1; i++)
            for (int j = i+1; j < histCount; j++)
                if (hist[j].count > hist[i].count)
                    { Entry t = hist[i]; hist[i] = hist[j]; hist[j] = t; }

        Serial.printf("[Palette] Total pixels: %d  transparent: %d  "
                      "unique colours: %d\n", total, transparent, histCount);
        Serial.println("[Palette] Top colours (RGB565 hex : count : R,G,B):");

        int show = min(histCount, 24);
        for (int i = 0; i < show; i++) {
            uint16_t c = hist[i].colour;
            // RGB565 -> R8,G8,B8
            uint8_t r = ((c >> 11) & 0x1F) << 3;
            uint8_t g = ((c >>  5) & 0x3F) << 2;
            uint8_t b = ( c        & 0x1F) << 3;
            Serial.printf("[Palette]   0x%04X : %5d :  R=%3d G=%3d B=%3d\n",
                          c, hist[i].count, r, g, b);
        }
        Serial.println("[Palette] Done. Use these values to build colour map.");
    }
#endif

public:
    // Called from main.cpp each loop to inject current time string
    void setTimeString(const char* t) {
        strncpy(_timeStr, t, sizeof(_timeStr) - 1);
        _timeStr[sizeof(_timeStr)-1] = '\0';
    }

private:
    char _timeStr[12] = "";
    int  _pngOffset   = 0;   // byte offset to PNG signature within _tileBuf
};