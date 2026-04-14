#pragma once
#include <LovyanGFX.hpp>

// ----------------------------------------------------------------
// JuliaRenderer
//
// Renders the Julia set scanline by scanline — no large heap
// allocation required. Works with or without PSRAM.
//
// Smooth (fractional) colouring with a data-driven palette.
// Because we can't do a two-pass normalize without buffering the
// whole frame, we use a fixed perceptually-scaled mapping instead,
// which looks good across all c values.
// ----------------------------------------------------------------

class JuliaRenderer {
public:
    static constexpr int   MAX_ITER  = 128;
    static constexpr float ESCAPE_R2 = 4.0f;

    // Viewport
    float xMin = -1.8f, xMax = 1.8f;
    float yMin = -1.2f, yMax = 1.2f;

    // Julia parameter c
    float cReal = -0.7f, cImag = 0.27f;

    void begin(LGFX* gfx) {
        _gfx = gfx;
        _w   = gfx->width();
        _h   = gfx->height();
        _buildPalette();
    }

    void render() {
        if (!_gfx) return;

        float xScale = (xMax - xMin) / (float)(_w - 1);
        float yScale = (yMax - yMin) / (float)(_h - 1);

        for (int py = 0; py < _h; py++) {
            float zi0 = yMin + py * yScale;

            for (int px = 0; px < _w; px++) {
                float zr = xMin + px * xScale;
                float zi = zi0;
                int   i  = 0;
                float r2 = 0.0f;

                while (i < MAX_ITER && (r2 = zr*zr + zi*zi) < ESCAPE_R2) {
                    float tmp = zr*zr - zi*zi + cReal;
                    zi = 2.0f * zr * zi + cImag;
                    zr = tmp;
                    i++;
                }

                uint16_t col;
                if (i == MAX_ITER) {
                    col = _interior;
                } else {
                    // Smooth iteration value
                    float smooth = (float)i - log2f(log2f(r2) * 0.5f);
                    // Map to [0,1] using log scale (looks good for all MAX_ITER)
                    float t = smooth / (float)MAX_ITER;
                    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
                    // Gamma curve to push detail into lower iteration counts
                    t = sqrtf(t);
                    col = _palette[(int)(t * 255.0f)];
                }
                _lineBuf[px] = col;
            }
            _gfx->pushImage(0, py, _w, 1, _lineBuf);
        }
    }

    void setParamFromTouch(uint16_t tx, uint16_t ty) {
        cReal = -1.5f + (float)tx / (float)(_w - 1) * 2.0f;
        cImag =  0.75f - (float)ty / (float)(_h - 1) * 1.5f;
    }

private:
    LGFX*    _gfx = nullptr;
    int      _w   = 480;
    int      _h   = 320;

    uint16_t _lineBuf[480];   // single scanline — only 960 bytes
    uint16_t _palette[256];
    uint16_t _interior = 0x0000;

    void _buildPalette() {
        // deep navy → cyan → gold → white
        for (int i = 0; i < 256; i++) {
            float t = i / 255.0f;
            uint8_t r, g, b;
            if (t < 0.33f) {
                float s = t / 0.33f;
                r = 0;
                g = (uint8_t)(s * 255);
                b = (uint8_t)(64 + s * 191);
            } else if (t < 0.66f) {
                float s = (t - 0.33f) / 0.33f;
                r = (uint8_t)(s * 255);
                g = (uint8_t)(255 - s * 51);
                b = (uint8_t)(255 - s * 255);
            } else {
                float s = (t - 0.66f) / 0.34f;
                r = 255;
                g = (uint8_t)(204 + s * 51);
                b = (uint8_t)(s * 255);
            }
            if (_gfx)
                _palette[i] = _gfx->color565(r, g, b);
        }
        if (_gfx)
            _interior = _gfx->color565(20, 20, 30);
    }
};