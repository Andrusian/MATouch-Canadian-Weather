#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "LovyanGFX_config.h"
#include "TileCalc.h"

// ----------------------------------------------------------------
// RadarMap — MATouch-Canadian-Weather
//
// Phase 1a: Fetch OSM basemap tiles → stitch into PSRAM sprite →
//           crop and blit 480×320 to display. Basemap cached in
//           _basemap sprite; only re-fetched on zoom change or boot.
//
// Phase 1b: Fetch MSC GeoMet WMS precipitation image → draw over
//           _composite sprite (copy of basemap) at RADAR_ALPHA →
//           push to display. Refreshed every RADAR_REFRESH_MS.
//
//           Source: https://geo.weather.gc.ca/geomet  (WMS 1.3.0)
//           Layers: RADAR_1KM_RRAI  (rain, 1 km, updated every 6 min)
//                   RADAR_1KM_RSNO  (snow, 1 km, updated every 6 min)
//           No API key required. No chunked-transfer quirks —
//           the WMS response is a plain PNG with Content-Length.
//
// Phase 1c: Lightning strike circles drawn over composite.
//           (unchanged from OWM version)
//
// Phase 2:  Centre-tap cycles zoom 400km → 100km → 50km → 400km.
//
// PSRAM budget (16bpp sprites):
//   _basemap   480×320 × 2 = 307,200 bytes  (~300 KB)
//   _composite 480×320 × 2 = 307,200 bytes  (~300 KB)
//   Tile fetch buffer       ≤ 50,000 bytes  (~ 50 KB, reused)
//   Total                                   (~650 KB of ~8 MB)
//
// MSC GeoMet WMS notes:
//   - No API key, anonymous access, free of charge.
//   - Uses WMS GetMap with BBOX in EPSG:4326 (lat/lon).
//   - We request one image sized to match the visible display area,
//     one per radar refresh cycle — simpler than XYZ tiling.
//   - Rain and snow are separate layers; we fetch both when snow
//     is possible (below SNOW_TEMP_C) and composite rain over snow.
//   - Default style omitted (server picks the official ECCC palette),
//     so _remapRadarColours() reinterprets that palette below.
//   - If TIME= is omitted the server returns the latest available
//     frame, which is what we want.
// ----------------------------------------------------------------

// Refresh interval for radar overlay (ms).
// 6 min matches the MSC update cadence exactly.
#define RADAR_REFRESH_MS  (6UL * 60UL * 1000UL)

// When to treat precipitation as snow vs rain.
// Set to an unreachable value (e.g. -99) to always fetch rain only.
#define SNOW_TEMP_C  2.0f

// Centre points per zoom level
#ifdef ALTERNATE
static const double CENTRE_LAT[3] = {
    40.75,    // z=7  400km
    40.75,    // z=9  100km
    40.75     // z=10  50km
};
static const double CENTRE_LON[3] = {
    -73.926,
    -73.926,
    -73.926
};
#else
static const double CENTRE_LAT[3] = {
    42.9574481183419,
    42.9574481183419,
    42.81111151466509      // z=10 — SSW of St Thomas
};
static const double CENTRE_LON[3] = {
    -81.16835498354426,
    -81.16835498354426,
    -81.25584709039006
};
#endif

// Display dimensions (post-rotation)
static constexpr int DISP_W = 480;
static constexpr int DISP_H = 320;

// Centre-tap detection zone (pixels from display centre)
static constexpr int TAP_CENTRE_ZONE = 60;

// PNG tile fetch buffer.
// MSC WMS images are typically 10-25 KB for sparse Canadian radar,
// occasionally up to ~40 KB for dense precipitation. 50 KB is safe.
static constexpr int TILE_BUF_SIZE = 50000;

// Alpha for radar overlay: 0=transparent, 255=opaque.
static constexpr uint8_t RADAR_ALPHA = 160;   // slightly more opaque
                                               // than OWM — MSC uses
                                               // a denser colour palette

// ----------------------------------------------------------------
// MSC GeoMet WMS URL builder
//
// Constructs a single GetMap request that covers the exact geographic
// extent of the current tile grid.  One HTTP request per radar
// refresh — much simpler than per-tile XYZ requests.
//
// Parameters:
//   buf      — output buffer
//   bufLen   — size of buf
//   layer    — WMS layer name, e.g. "RADAR_1KM_RRAI"
//   latMin, lonMin, latMax, lonMax — bounding box in EPSG:4326
//   imgW, imgH — requested image pixel dimensions
// ----------------------------------------------------------------
static void _mscWmsUrl(char* buf, size_t bufLen,
                       const char* layer,
                       double latMin, double lonMin,
                       double latMax, double lonMax,
                       int imgW, int imgH)
{
    // EPSG:4326 axis order for WMS 1.3.0 is (latitude, longitude),
    // so BBOX = minLat,minLon,maxLat,maxLon.
    snprintf(buf, bufLen,
        "https://geo.weather.gc.ca/geomet"
        "?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap"
        "&LAYERS=%s"
        "&CRS=EPSG:4326"
        "&BBOX=%.6f,%.6f,%.6f,%.6f"
        "&WIDTH=%d&HEIGHT=%d"
        "&FORMAT=image/png"
        "&TRANSPARENT=TRUE",
        layer,
        latMin, lonMin, latMax, lonMax,
        imgW, imgH);
}

// ----------------------------------------------------------------
// Geographic extent of a TileGrid
//
// TileCalc gives us integer tile coordinates; we need the lat/lon
// bounding box of the *cropped* display window for the WMS BBOX.
// These helpers convert OSM tile numbers to lat/lon.
// ----------------------------------------------------------------
static double _tileLon(int tx, int z) {
    return tx / (double)(1 << z) * 360.0 - 180.0;
}
static double _tileLat(int ty, int z) {
    double n = M_PI - 2.0 * M_PI * ty / (double)(1 << z);
    return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

// Given a TileGrid, compute the lat/lon extents of the display
// viewport (the cropped 480×320 window, not the full canvas).
static void _gridBbox(const TileGrid& g,
                      double& latMin, double& lonMin,
                      double& latMax, double& lonMax)
{
    // Pixel offset of the top-left display corner within the canvas
    double pxLeft  = g.cropX;
    double pxTop   = g.cropY;
    double pxRight = g.cropX + DISP_W;
    double pxBot   = g.cropY + DISP_H;

    // Convert pixel offsets to fractional tile coordinates,
    // then to lat/lon using standard OSM tile maths.
    // tileX0 / tileY0 are the integer tile indices of the first tile;
    // each tile is 256 px wide.
    double txLeft  = g.tileX0 + pxLeft  / 256.0;
    double txRight = g.tileX0 + pxRight / 256.0;
    double tyTop   = g.tileY0 + pxTop   / 256.0;
    double tyBot   = g.tileY0 + pxBot   / 256.0;

    lonMin = txLeft  / (double)(1 << g.z) * 360.0 - 180.0;
    lonMax = txRight / (double)(1 << g.z) * 360.0 - 180.0;

    // lat from Mercator: larger tyBot → smaller lat (south)
    auto mercLat = [&](double ty_frac) -> double {
        double n = M_PI - 2.0 * M_PI * ty_frac / (double)(1 << g.z);
        return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
    };
    latMax = mercLat(tyTop);
    latMin = mercLat(tyBot);
}


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
    // ------------------------------------------------------------------
    bool begin() {
        Serial.printf("[RadarMap] Free internal RAM: %u\n",
                      heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        Serial.printf("[RadarMap] Free PSRAM before alloc: %u\n",
                      ESP.getFreePsram());

        _basemap = new lgfx::LGFX_Sprite(_display);
        _basemap->setPsram(true);
        if (!_basemap->createSprite(DISP_W, DISP_H)) {
            Serial.println("[RadarMap] ERROR: basemap sprite alloc failed");
            return false;
        }
        _basemap->fillScreen(TFT_BLACK);

        _composite = new lgfx::LGFX_Sprite(_display);
        _composite->setPsram(true);
        if (!_composite->createSprite(DISP_W, DISP_H)) {
            Serial.println("[RadarMap] ERROR: composite sprite alloc failed");
            return false;
        }
        _composite->fillScreen(TFT_BLACK);

        // Internal RAM required — pngle cannot safely access PSRAM.
        _tileBuf = (uint8_t*)heap_caps_malloc(TILE_BUF_SIZE, MALLOC_CAP_INTERNAL);
        if (!_tileBuf) {
            Serial.printf("[RadarMap] Internal RAM alloc failed, free: %u\n",
                          heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
            return false;
        }

        Serial.printf("[RadarMap] Free PSRAM after alloc:  %u\n",
                      ESP.getFreePsram());

        _fetchBasemap();
        _lastRadarMs = millis() - RADAR_REFRESH_MS;  // immediate first fetch
        return true;
    }

    // ------------------------------------------------------------------
    // update() — call every loop() iteration.
    // ------------------------------------------------------------------
    void update(bool dimmed) {
        if (dimmed) return;
        uint32_t now = millis();
        if ((now - _lastRadarMs) >= RADAR_REFRESH_MS) {
            _lastRadarMs = now;
            _buildComposite();
            _pushToDisplay();
        }
    }

    // ------------------------------------------------------------------
    // onTouch() — centre-tap cycles zoom level.
    // ------------------------------------------------------------------
    bool onTouch(uint16_t tx, uint16_t ty) {
        int dx = (int)tx - DISP_W / 2;
        int dy = (int)ty - DISP_H / 2;
        if (abs(dx) > TAP_CENTRE_ZONE || abs(dy) > TAP_CENTRE_ZONE)
            return false;

        _zoomIndex = (_zoomIndex + 1) % ZOOM_LEVEL_COUNT;
        Serial.printf("[RadarMap] Zoom -> index %d  z=%d\n",
                      _zoomIndex, ZOOM_LEVELS[_zoomIndex]);

        _basemapValid = false;
        _fetchBasemap();
        _lastRadarMs = millis() - RADAR_REFRESH_MS;
        return true;
    }

    const char* zoomLabel() const {
        switch (_zoomIndex) {
            case 0: return "400km";
            case 1: return "100km";
            default: return " 50km";
        }
    }

    void setTimeString(const char* t) {
        strncpy(_timeStr, t, sizeof(_timeStr) - 1);
        _timeStr[sizeof(_timeStr)-1] = '\0';
    }

private:
    LGFX*              _display;
    lgfx::LGFX_Sprite* _basemap;
    lgfx::LGFX_Sprite* _composite;
    uint8_t*           _tileBuf;

    int      _zoomIndex;
    bool     _basemapValid;
    uint32_t _lastRadarMs;
    char     _timeStr[12] = "";
    int      _pngOffset   = 0;

    // ------------------------------------------------------------------
    // _currentGrid()
    // ------------------------------------------------------------------
    TileGrid _currentGrid() const {
        int z = ZOOM_LEVELS[_zoomIndex];
        return TileCalc::tileGrid(
            CENTRE_LAT[_zoomIndex],
            CENTRE_LON[_zoomIndex],
            z, DISP_W, DISP_H);
    }

    // ------------------------------------------------------------------
    // _fetchBasemap() — OSM tiles (unchanged from OWM version)
    // ------------------------------------------------------------------
    void _fetchBasemap() {
        TileGrid g = _currentGrid();
        TileCalc::printGrid(g, zoomLabel());

        _basemap->fillScreen(TFT_DARKGREY);

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

        int serial = 0, fetched = 0;
        for (int row = 0; row < g.rowCount; row++) {
            for (int col = 0; col < g.colCount; col++) {
                int cx = col * 256;
                int cy = row * 256;
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
                    canvas.fillRect(cx+1, cy+1, 254, 254, TFT_DARKGREY);
                    canvas.drawRect(cx, cy, 256, 256, TFT_RED);
                }
                delay(50);
            }
        }

        uint32_t elapsed = millis() - t0;
        Serial.printf("[RadarMap] Basemap: %d/%d tiles in %lu ms\n",
                      fetched, totalTiles, elapsed);

        canvas.pushSprite(_basemap, -g.cropX, -g.cropY);
        canvas.deleteSprite();

        _drawScaleBar(_basemap, ZOOM_LEVELS[_zoomIndex],
                      CENTRE_LAT[_zoomIndex]);
        _drawZoomLabel(_basemap);

        _basemapValid = true;
        Serial.println("[RadarMap] Basemap ready.");
    }

    // ------------------------------------------------------------------
    // _buildComposite() — copy basemap, then overlay MSC GeoMet radar.
    //
    // MSC provides separate rain and snow layers.  We issue one WMS
    // GetMap request per active layer, sized to the display viewport.
    // The server returns a PNG at the exact pixel dimensions we ask for,
    // so no per-tile stitching loop is needed.
    //
    // Layer selection:
    //   Always fetch RADAR_1KM_RRAI (rain).
    //   Fetch RADAR_1KM_RSNO (snow) when temperature ≤ SNOW_TEMP_C.
    //   Snow is composited first (under rain).
    // ------------------------------------------------------------------

    
    void _buildComposite() {
        if (!_basemapValid) {
            Serial.println("[RadarMap] Basemap not valid, fetching...");
            _fetchBasemap();
        }

        _basemap->pushSprite(_composite, 0, 0);

        TileGrid g = _currentGrid();

        // Compute the geographic bounding box of our display viewport
        double latMin, lonMin, latMax, lonMax;
        _gridBbox(g, latMin, lonMin, latMax, lonMax);
        Serial.printf("[RadarMap] WMS BBOX: %.4f,%.4f,%.4f,%.4f  z=%d\n",
                      latMin, lonMin, latMax, lonMax, g.z);

        // -- Snow layer: commented out pending investigation --
        // Second/third HTTP requests return no data; investigate separately.
        // {
        //     char url[512];
        //     _mscWmsUrl(url, sizeof(url), "RADAR_1KM_RSNO",
        //                latMin, lonMin, latMax, lonMax, DISP_W, DISP_H);
        //     int bytes = _fetchPng(url);
        //     if (bytes > 0) {
        //         _overlayRadarImage(_tileBuf+_pngOffset, bytes-_pngOffset,
        //                            0, 0, DISP_W, DISP_H, true);
        //     }
        // }

        // -- Rain layer --
        {
            char url[512];
            _mscWmsUrl(url, sizeof(url), "RADAR_1KM_RRAI",
                       latMin, lonMin, latMax, lonMax,
                       DISP_W, DISP_H);
            Serial.println("[RadarMap] Fetching MSC rain radar...");
            int bytes = _fetchPng(url);
            if (bytes > 0) {
                int pngLen = bytes - _pngOffset;
                _overlayRadarImage(_tileBuf + _pngOffset, pngLen,
                                   0, 0, DISP_W, DISP_H,
                                   /*isSnow=*/false);
            } else {
                Serial.println("[RadarMap] Rain radar: no data");
            }
        }

        // UI overlays
        _drawScaleBar(_composite, ZOOM_LEVELS[_zoomIndex],
                      CENTRE_LAT[_zoomIndex]);
        _drawZoomLabel(_composite);
        _drawTimestamp(_composite);
    }

    // ------------------------------------------------------------------
    
void _overlayRadarImage(const uint8_t* data, size_t len, 
                        int x, int y, int w, int h, 
                        bool isSnow) 
{
    lgfx::LGFX_Sprite tempRadar(_display);
    tempRadar.setPsram(true);
    
    if (!tempRadar.createSprite(w, h)) 
    {
        return;
    }

    // Using 0x0001 as a chroma key to identify transparent areas
    uint32_t chromaKey = 0x000001; 
    tempRadar.fillScreen(chromaKey);
    tempRadar.drawPng(data, len, 0, 0);

    for (int py = 0; py < h; py++) 
    {
        for (int px = 0; px < w; px++) 
        {
            uint16_t rawCol = tempRadar.readPixel(px, py);
            
            if (rawCol == chromaKey || rawCol == 0x0000) 
            {
                continue;
            }

            // Convert RGB565 to 888 for easier thresholding
            uint8_t r = ((rawCol >> 11) & 0x1F) << 3;
            uint8_t g = ((rawCol >>  5) & 0x3F) << 2;
            uint8_t b = ( rawCol        & 0x1F) << 3;

            uint16_t displayCol;
            
            if (isSnow) 
            {
                displayCol = 0x94B2; // Muted Blue/Gray
            }
            else 
            {
                // MSC Light Rain: Cyan (High Blue & Green, Low Red)
                if (b > 200 && g > 200 && r < 150) 
                {
                    displayCol = 0x5EBF; // Light Blue
                } 
                else if (g > 180 && r < 150) 
                {
                    displayCol = 0x07E0; // Green
                } 
                else if (r > 180 && g < 150) 
                {
                    displayCol = 0xF800; // Red
                } 
                else 
                {
                    displayCol = rawCol; 
                }
            }

            uint16_t bgCol = _composite->readPixel(x + px, y + py);
            
            // Using display method to blend: alpha is 0-255
            // 128 is roughly 50% transparency
            // Manual 50/50 blend for RGB565
            uint16_t blended = ((displayCol & 0xF7DE) >> 1) + ((bgCol & 0xF7DE) >> 1);
            //uint16_t blended = _display->colorAlpha(_display->alphaBlend(128, displayCol, bgCol));
            
            // Note: If alphaBlend still complains, we can use the manual bit-shift 
            // math, but let's try the built-in one first.
            _composite->drawPixel(x + px, y + py, blended);
        }
    }

    tempRadar.deleteSprite();
}


    // ------------------------------------------------------------------
    // _pushToDisplay()
    // ------------------------------------------------------------------
    void _pushToDisplay() {
        _composite->pushSprite(0, 0);
    }

    // ------------------------------------------------------------------
    // _fetchPng() — fetch a PNG URL into _tileBuf.
    // Returns total bytes read (>0) or 0 on failure.
    //
    // MSC GeoMet returns plain PNG with Content-Length (no chunking).
    // The chunked path is kept as a fallback in case of proxy behaviour.
    // ------------------------------------------------------------------
    int _fetchPng(const char* url) {
        HTTPClient http;
        WiFiClientSecure secureClient;

        bool isHttps = (strncmp(url, "https://", 8) == 0);
        if (isHttps) {
            secureClient.setInsecure();
            http.begin(secureClient, url);
        } else {
            http.begin(url);
        }
        Serial.printf("URL: %s\n",url);

        http.setTimeout(15000);
        http.setUserAgent("MaTouch-RadarMap/1.0");

        int code = http.GET();
        int len  = http.getSize();
        Serial.printf("[RadarMap] HTTP %d  len=%d\n", code, len);
        if (code != HTTP_CODE_OK) {
            http.end();
            return 0;
        }

        if (len > TILE_BUF_SIZE) {
            Serial.printf("[RadarMap] Response too large: %d bytes\n", len);
            http.end();
            return 0;
        }

        int maxRead = (len > 0) ? min(len, TILE_BUF_SIZE) : TILE_BUF_SIZE;
        WiFiClient* stream = http.getStreamPtr();
        stream->setTimeout(10000);

        int bytesRead = 0;

        if (len > 0) {
            // MSC GeoMet normal path: Content-Length present
            bytesRead = (int)stream->readBytes(_tileBuf, maxRead);
        } else {
            // Chunked fallback (kept from OWM version)
            uint32_t deadline = millis() + 15000;
            while (bytesRead < maxRead && millis() < deadline) {
                char sizeLine[16] = {0};
                int si = 0;
                uint32_t lineDeadline = millis() + 3000;
                while (millis() < lineDeadline && si < 15) {
                    if (stream->available()) {
                        char c = stream->read();
                        if (c == '\n') break;
                        if (c != '\r') sizeLine[si++] = c;
                    } else delay(1);
                }
                sizeLine[si] = 0;
                int chunkSize = (int)strtol(sizeLine, nullptr, 16);
                if (chunkSize == 0) break;
                int got = (int)stream->readBytes(
                    _tileBuf + bytesRead,
                    min(chunkSize, maxRead - bytesRead));
                bytesRead += got;
                uint32_t crDeadline = millis() + 1000;
                int crCount = 0;
                while (crCount < 2 && millis() < crDeadline) {
                    if (stream->available()) { stream->read(); crCount++; }
                    else delay(1);
                }
            }
        }

        http.end();

        if (bytesRead == 0) {
            Serial.println("[RadarMap] No data received");
            return 0;
        }
        Serial.printf("[RadarMap] Read %d bytes\n", bytesRead);

        // Locate PNG signature
        int pngOffset = -1;
        for (int i = 0; i <= min(bytesRead - 4, 16); i++) {
            if (_tileBuf[i]   == 0x89 && _tileBuf[i+1] == 'P' &&
                _tileBuf[i+2] == 'N'  && _tileBuf[i+3] == 'G') {
                pngOffset = i;
                break;
            }
        }
        if (pngOffset < 0) {
            Serial.printf("[RadarMap] No PNG signature (%d bytes). First bytes:\n",
                          bytesRead);
            int show = min(bytesRead, 64);
            for (int i = 0; i < show; i++) {
                char c = (char)_tileBuf[i];
                Serial.print((c >= 32 && c < 127) ? c : '.');
            }
            Serial.println();
            return 0;
        }
        if (pngOffset > 0)
            Serial.printf("[RadarMap] PNG at offset %d\n", pngOffset);

        _pngOffset = pngOffset;
        return bytesRead;
    }

    // ------------------------------------------------------------------
    // _remapRadarColours()
    //
    // MSC GeoMet RADAR_1KM_RRAI uses the default RADARURPPRECIPR14 style
    // (dBZ-based, 14 colour steps).  The dominant colours observed in
    // the decoded PNG (after RGB565 conversion) are:
    //
    //   Very light / ND  — near-black, score < 30          → suppress
    //   Light green      — low dBZ (drizzle) score 30–90   → COL_LGREEN
    //   Green            — moderate score 90–150            → COL_GREEN
    //   Yellow           — heavy score 150–190              → COL_YELLOW
    //   Orange/red       — very heavy 190–230               → COL_ORANGE
    //   Red/purple       — severe 230+                      → COL_RED / COL_PURPLE
    //
    // Snow (RADAR_1KM_RSNO) uses the same dBZ scale but is tinted cyan
    // in the official palette; we remap to a blue family so rain and
    // snow are visually distinct on the display.
    //
    // IMPORTANT: Run once with DEBUG defined to sample the actual palette
    // from your area and fine-tune these thresholds.  The MSC palette is
    // quite different from OWM's — in particular there is no uniform
    // near-black "no precip" value; truly no-precip pixels are rendered
    // transparent in the WMS PNG (alpha=0), which drawPng() maps to
    // black (0x0000) in the sprite, so our "skip 0x0000" test still works.
    // ------------------------------------------------------------------
    // Single-pixel remap — returns 0x0000 if the pixel should be suppressed.
    uint16_t _remapOnePixel(uint16_t px, bool isSnow) {
        if (px == 0x0000) return 0x0000;
        uint8_t r = ((px >> 11) & 0x1F) << 3;
        uint8_t g = ((px >>  5) & 0x3F) << 2;
        uint8_t b = ( px        & 0x1F) << 3;
        int score = r * 3 + g * 2 + b;

        static constexpr int SUPPRESS_BELOW = 1;
        static constexpr int THRESH_LIGHT   = 90;
        static constexpr int THRESH_MEDIUM  = 150;
        static constexpr int THRESH_HEAVY   = 190;
        static constexpr int THRESH_SEVERE  = 230;

        if (score < SUPPRESS_BELOW) return 0x0000;

        if (!isSnow) {
            static constexpr uint16_t RAIN_LGREEN = 0x2D40;
            static constexpr uint16_t RAIN_GREEN  = 0x04E0;
            static constexpr uint16_t RAIN_YELLOW = 0xFFE0;
            static constexpr uint16_t RAIN_ORANGE = 0xFC40;
            static constexpr uint16_t RAIN_RED    = 0xF800;
            static constexpr uint16_t RAIN_PURPLE = 0x8010;
            if      (score < THRESH_LIGHT)  return RAIN_LGREEN;
            else if (score < THRESH_MEDIUM) return RAIN_GREEN;
            else if (score < THRESH_HEAVY)  return RAIN_YELLOW;
            else if (score < THRESH_SEVERE) return RAIN_ORANGE;
            else if (score < 250)           return RAIN_RED;
            else                            return RAIN_PURPLE;
        } else {
            static constexpr uint16_t SNOW_LTBLUE = 0x867F;
            static constexpr uint16_t SNOW_BLUE   = 0x041F;
            static constexpr uint16_t SNOW_BLUWHT = 0xAEFF;
            static constexpr uint16_t SNOW_WHITE  = 0xFFFF;
            static constexpr uint16_t SNOW_PINK   = 0xF81F;
            if      (score < THRESH_LIGHT)  return SNOW_LTBLUE;
            else if (score < THRESH_MEDIUM) return SNOW_BLUE;
            else if (score < THRESH_HEAVY)  return SNOW_BLUWHT;
            else if (score < THRESH_SEVERE) return SNOW_WHITE;
            else                            return SNOW_PINK;
        }
    }

    void _remapRadarColours(lgfx::LGFX_Sprite* spr, int w, int h, bool isSnow) {
        // Score thresholds (R*3 + G*2 + B on RGB565 expanded values)
        static constexpr int SUPPRESS_BELOW = 30;
        static constexpr int THRESH_LIGHT   = 90;
        static constexpr int THRESH_MEDIUM  = 150;
        static constexpr int THRESH_HEAVY   = 190;
        static constexpr int THRESH_SEVERE  = 230;

        // Rain palette (green → yellow → orange → red → purple)
        static constexpr uint16_t RAIN_LGREEN = 0x2D40;  // #2B6800 dark green
        static constexpr uint16_t RAIN_GREEN  = 0x04E0;  // #00980 green
        static constexpr uint16_t RAIN_YELLOW = 0xFFE0;  // yellow
        static constexpr uint16_t RAIN_ORANGE = 0xFC40;  // orange
        static constexpr uint16_t RAIN_RED    = 0xF800;  // red
        static constexpr uint16_t RAIN_PURPLE = 0x8010;  // purple

        // Snow palette (light blue → blue → blue-white → white → magenta)
        static constexpr uint16_t SNOW_LTBLUE = 0x867F;  // light blue
        static constexpr uint16_t SNOW_BLUE   = 0x041F;  // blue
        static constexpr uint16_t SNOW_BLUWHT = 0xAEFF;  // blue-white
        static constexpr uint16_t SNOW_WHITE  = 0xFFFF;  // white
        static constexpr uint16_t SNOW_PINK   = 0xF81F;  // magenta (heavy snow)

        int remapped = 0, suppressed = 0;

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                uint16_t px = spr->readPixel(x, y);
                if (px == 0x0000) continue;  // transparent — leave as is

                uint8_t r = ((px >> 11) & 0x1F) << 3;
                uint8_t g = ((px >>  5) & 0x3F) << 2;
                uint8_t b = ( px        & 0x1F) << 3;
                int score = r * 3 + g * 2 + b;

                if (score < SUPPRESS_BELOW) {
                    spr->drawPixel(x, y, 0x0000);
                    suppressed++;
                    continue;
                }

                uint16_t newCol;
                if (!isSnow) {
                    if      (score < THRESH_LIGHT)  newCol = RAIN_LGREEN;
                    else if (score < THRESH_MEDIUM) newCol = RAIN_GREEN;
                    else if (score < THRESH_HEAVY)  newCol = RAIN_YELLOW;
                    else if (score < THRESH_SEVERE) newCol = RAIN_ORANGE;
                    else if (score < 250)           newCol = RAIN_RED;
                    else                            newCol = RAIN_PURPLE;
                } else {
                    if      (score < THRESH_LIGHT)  newCol = SNOW_LTBLUE;
                    else if (score < THRESH_MEDIUM) newCol = SNOW_BLUE;
                    else if (score < THRESH_HEAVY)  newCol = SNOW_BLUWHT;
                    else if (score < THRESH_SEVERE) newCol = SNOW_WHITE;
                    else                            newCol = SNOW_PINK;
                }

                spr->drawPixel(x, y, newCol);
                remapped++;
            }
        }
        Serial.printf("[RadarMap]   remap(%s): %d px remapped, %d suppressed\n",
                      isSnow ? "snow" : "rain", remapped, suppressed);
    }

    // ------------------------------------------------------------------
    // UI overlays (unchanged from OWM version)
    // ------------------------------------------------------------------
    void _drawScaleBar(lgfx::LGFX_Sprite* spr, int z, double lat) {
        float kpp = TileCalc::kmPerPixel(lat, z);
        float targets[] = { 200, 100, 50, 25, 10, 5 };
        float barKm = targets[0];
        for (float t : targets) {
            if (t / kpp < 120) { barKm = t; break; }
        }
        int barPx = (int)(barKm / kpp);
        int x0 = 10, y0 = DISP_H - 18;
        int x1 = x0 + barPx;
        spr->drawFastHLine(x0+1, y0+1, barPx, TFT_BLACK);
        spr->drawFastVLine(x0+1, y0+1, 6,     TFT_BLACK);
        spr->drawFastVLine(x1+1, y0+1, 6,     TFT_BLACK);
        spr->drawFastHLine(x0,   y0,   barPx, TFT_WHITE);
        spr->drawFastVLine(x0,   y0,   6,     TFT_WHITE);
        spr->drawFastVLine(x1,   y0,   6,     TFT_WHITE);
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

    void _drawZoomLabel(lgfx::LGFX_Sprite* spr) {
        spr->setTextSize(1);
        spr->setTextColor(TFT_BLACK, TFT_TRANSPARENT);
        spr->setCursor(9, 9);
        spr->print(zoomLabel());
        spr->setTextColor(TFT_WHITE, TFT_TRANSPARENT);
        spr->setCursor(8, 8);
        spr->print(zoomLabel());
    }

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

#ifdef DEBUG
    // ------------------------------------------------------------------
    // _sampleTilePalette() — diagnostic: print colour histogram to Serial.
    // Generalised version works for any sprite size (not just 256×256).
    // ------------------------------------------------------------------
    void _sampleTilePalette(lgfx::LGFX_Sprite* spr, int w, int h) {
        Serial.printf("[Palette] Sampling %dx%d radar image...\n", w, h);
        struct Entry { uint16_t colour; int count; };
        static Entry hist[64];
        int histCount = 0;
        int transparent = 0, total = 0;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                uint16_t px = spr->readPixel(x, y);
                total++;
                if (px == 0x0000) { transparent++; continue; }
                bool found = false;
                for (int i = 0; i < histCount; i++) {
                    if (hist[i].colour == px) { hist[i].count++; found = true; break; }
                }
                if (!found && histCount < 64)
                    hist[histCount++] = {px, 1};
            }
        }
        for (int i = 0; i < histCount - 1; i++)
            for (int j = i+1; j < histCount; j++)
                if (hist[j].count > hist[i].count)
                    { Entry t = hist[i]; hist[i] = hist[j]; hist[j] = t; }
        Serial.printf("[Palette] Total: %d  transparent: %d  unique: %d\n",
                      total, transparent, histCount);
        int show = min(histCount, 24);
        for (int i = 0; i < show; i++) {
            uint16_t c = hist[i].colour;
            uint8_t r = ((c >> 11) & 0x1F) << 3;
            uint8_t g = ((c >>  5) & 0x3F) << 2;
            uint8_t b = ( c        & 0x1F) << 3;
            Serial.printf("[Palette]   0x%04X : %5d :  R=%3d G=%3d B=%3d\n",
                          c, hist[i].count, r, g, b);
        }
        Serial.println("[Palette] Done.");
    }
#endif
};