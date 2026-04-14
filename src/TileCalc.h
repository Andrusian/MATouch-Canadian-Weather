#pragma once
#include <Arduino.h>
#include <math.h>

// ----------------------------------------------------------------
// TileCalc — OSM slippy-map tile math
//
// All functions are pure (no I/O, no hardware) so they can be
// verified by Serial.printf before any drawing happens.
//
// Coordinate convention:
//   Tile (tx, ty) at zoom z is a 256×256 px image.
//   Pixel (0,0) is top-left; x increases east, y increases south.
//
// Reference: https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
// ----------------------------------------------------------------

struct TileCoord {
    int    z;       // zoom level
    int    tx;      // tile column
    int    ty;      // tile row
    int    px;      // pixel column of target point within tile (0-255)
    int    py;      // pixel row    of target point within tile (0-255)
};

// Grid descriptor: which tiles to fetch and how to crop them
// onto a 480×320 display centered on the target point.
struct TileGrid {
    int z;

    // Top-left tile of the fetch grid
    int tileX0;     // leftmost tile column
    int tileY0;     // topmost  tile row

    // Grid dimensions in tiles
    int colCount;   // number of tile columns to fetch (2 or 3)
    int rowCount;   // number of tile rows    to fetch (2 or 3)

    // Pixel offset into the stitched canvas to start the 480×320 crop.
    // stitched canvas size = colCount*256 × rowCount*256
    int cropX;      // left edge of display window in stitched canvas
    int cropY;      // top  edge of display window in stitched canvas

    // Sanity: total stitched canvas dimensions
    int canvasW() const { return colCount * 256; }
    int canvasH() const { return rowCount * 256; }
};

// ----------------------------------------------------------------
// Zoom levels for the three distance ranges.
// At your latitude (~43°N) one 256-px tile is approximately:
//   z=7  → ~313 km wide   (covers ~400 km view with 2 tiles)
//   z=9  → ~78  km wide   (covers ~100 km view with 2 tiles)
//   z=10 → ~39  km wide   (covers ~50  km view with 2 tiles)
// ----------------------------------------------------------------
static constexpr int ZOOM_400KM = 7;
static constexpr int ZOOM_100KM = 9;
static constexpr int ZOOM_50KM  = 10;

// Cycle order for center-tap gesture
static constexpr int ZOOM_LEVELS[]    = { ZOOM_400KM, ZOOM_100KM, ZOOM_50KM };
static constexpr int ZOOM_LEVEL_COUNT = 3;

class TileCalc {
public:

    // ------------------------------------------------------------------
    // lon2tx / lat2ty — fractional tile position of a geographic point.
    // The integer part is the tile index; fractional part * 256 = pixel.
    // ------------------------------------------------------------------
    static double lon2tx(double lon, int z) {
        return (lon + 180.0) / 360.0 * (1 << z);
    }

    static double lat2ty(double lat, int z) {
        double latRad = lat * DEG_TO_RAD;
        return (1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / M_PI) / 2.0 * (1 << z);
    }

    // ------------------------------------------------------------------
    // tileCoord — compute tile + intra-tile pixel for a lat/lon at zoom z
    // ------------------------------------------------------------------
    static TileCoord tileCoord(double lat, double lon, int z) {
        double ftx = lon2tx(lon, z);
        double fty = lat2ty(lat, z);

        TileCoord tc;
        tc.z  = z;
        tc.tx = (int)ftx;
        tc.ty = (int)fty;
        tc.px = (int)((ftx - tc.tx) * 256.0);
        tc.py = (int)((fty - tc.ty) * 256.0);
        return tc;
    }

    // ------------------------------------------------------------------
    // tileGrid — build the minimal fetch grid so that the target point
    // lands at the centre of a 480×320 display.
    //
    // Strategy:
    //   Centre pixel on display = (240, 160).
    //   Centre pixel in stitched canvas = cropX + 240, cropY + 160.
    //   We want that to coincide with (tc.tx * 256 + tc.px, tc.ty * 256 + tc.py)
    //   relative to the top-left tile of the grid.
    //
    //   Minimum grid with 2×2 tiles gives a 512×512 canvas.
    //   If the intra-tile pixel offset is too close to an edge
    //   (< 240 px from tile-column left, or < 160 px from tile-row top)
    //   we need an extra column or row on that side.
    // ------------------------------------------------------------------
    static TileGrid tileGrid(double lat, double lon, int z,
                             int dispW = 480, int dispH = 320) {
        TileCoord tc = tileCoord(lat, lon, z);

        // Pixel position of the target point within the FIRST tile column/row
        // if we start the grid at tc.tx / tc.ty.
        // We need (halfW) pixels to the left and (halfH) pixels above centre.
        int halfW = dispW / 2;   // 240
        int halfH = dispH / 2;   // 160

        // How many columns do we need?
        // Left of centre: tc.px pixels available in the anchor tile column.
        // We need halfW pixels left of centre.
        int leftTiles  = (tc.px < halfW) ? 1 : 0;  // extra col on left
        int rightPixels = (leftTiles ? tc.px + 256 : tc.px);  // pixels to right edge of anchor col
        int colCount = leftTiles + 1 +
                       ((rightPixels - halfW) < (dispW - halfW) ? 1 : 0);
        // Ensure we always have at least 2 columns
        if (colCount < 2) colCount = 2;
        // Cap — 3 should be enough for any of our zoom levels on a 480-wide display
        if (colCount > 3) colCount = 3;

        // Same for rows
        int topTiles   = (tc.py < halfH) ? 1 : 0;
        int bottomPixels = (topTiles ? tc.py + 256 : tc.py);
        int rowCount = topTiles + 1 +
                       ((bottomPixels - halfH) < (dispH - halfH) ? 1 : 0);
        if (rowCount < 2) rowCount = 2;
        if (rowCount > 3) rowCount = 3;

        // Top-left tile of the grid
        int tileX0 = tc.tx - leftTiles;
        int tileY0 = tc.ty - topTiles;

        // Pixel position of the target point within the stitched canvas
        int targetCanvasX = leftTiles * 256 + tc.px;
        int targetCanvasY = topTiles  * 256 + tc.py;

        // Crop origin so that target point lands at display centre
        int cropX = targetCanvasX - halfW;
        int cropY = targetCanvasY - halfH;

        // Clamp (shouldn't be needed if colCount/rowCount are correct,
        // but guards against edge cases)
        if (cropX < 0) cropX = 0;
        if (cropY < 0) cropY = 0;
        int maxCropX = colCount * 256 - dispW;
        int maxCropY = rowCount * 256 - dispH;
        if (cropX > maxCropX) cropX = maxCropX;
        if (cropY > maxCropY) cropY = maxCropY;

        TileGrid g;
        g.z        = z;
        g.tileX0   = tileX0;
        g.tileY0   = tileY0;
        g.colCount = colCount;
        g.rowCount = rowCount;
        g.cropX    = cropX;
        g.cropY    = cropY;
        return g;
    }

    // ------------------------------------------------------------------
    // Dump a TileGrid to Serial for verification
    // ------------------------------------------------------------------
    static void printGrid(const TileGrid& g, const char* label = "") {
        Serial.printf("[TileCalc] %s z=%d  tiles=(%d,%d) grid=%dx%d"
                      "  canvas=%dx%d  crop=(%d,%d)\n",
                      label, g.z, g.tileX0, g.tileY0,
                      g.colCount, g.rowCount,
                      g.canvasW(), g.canvasH(),
                      g.cropX, g.cropY);

        // Print each tile that will be fetched
        for (int row = 0; row < g.rowCount; row++) {
            for (int col = 0; col < g.colCount; col++) {
                int tx = g.tileX0 + col;
                int ty = g.tileY0 + row;
                // Destination rect in stitched canvas
                int cx = col * 256;
                int cy = row * 256;
                // Destination rect on display after crop
                int dx = cx - g.cropX;
                int dy = cy - g.cropY;
                Serial.printf("[TileCalc]   tile[%d,%d] -> OSM(%d,%d,%d)"
                              "  canvas(%d,%d)  display(%d,%d)\n",
                              col, row, g.z, tx, ty, cx, cy, dx, dy);
            }
        }
    }

    // ------------------------------------------------------------------
    // Build OSM basemap tile URL
    // Rotates between a/b/c subdomains to be polite to OSM servers
    // ------------------------------------------------------------------
    static void osmUrl(char* buf, size_t len,
                       int z, int tx, int ty, int serial = 0) {
        static const char subdomains[] = "abc";
        char sub = subdomains[serial % 3];
        snprintf(buf, len,
                 "https://%c.tile.openstreetmap.org/%d/%d/%d.png",
                 sub, z, tx, ty);
    }

    // ------------------------------------------------------------------
    // Build OWM radar tile URL
    // layer: "precipitation_new", "clouds_new", "wind_new" etc.
    // ------------------------------------------------------------------
    static void owmUrl(char* buf, size_t len,
                       const char* layer, int z, int tx, int ty,
                       const char* apiKey) {
        snprintf(buf, len,
                 "https://tile.openweathermap.org/map/%s/%d/%d/%d.png?appid=%s",
                 layer, z, tx, ty, apiKey);
    }

    // ------------------------------------------------------------------
    // Approximate km-per-pixel at a given latitude and zoom level.
    // Useful for scale bar display.
    // ------------------------------------------------------------------
    static float kmPerPixel(double lat, int z) {
        // Earth circumference at equator = 40075 km
        // At zoom z, one tile = 256 px covers 40075/2^z km at equator
        // Scale by cos(lat) for latitude correction
        double latRad = lat * DEG_TO_RAD;
        return (float)(40075.0 * cos(latRad) / (256.0 * (1 << z)));
    }
};