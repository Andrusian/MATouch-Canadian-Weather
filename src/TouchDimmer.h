#pragma once
#include <Arduino.h>
#include <functional>

// Manages screen dimming:
//   - Double-tap lower-left corner (within CORNER_PX, within DOUBLE_TAP_MS)
//     toggles dim/bright. First tap on a dimmed screen always just wakes it.
//   - Auto-dims after AUTO_DIM_MS of no touch activity.

class TouchDimmer {
public:
    static constexpr uint8_t  BRIGHTNESS_FULL  = 255;
    static constexpr uint8_t  BRIGHTNESS_DIM   = 25;       // ~10% — tweak to taste
    static constexpr uint32_t DOUBLE_TAP_MS    = 500;
    static constexpr uint16_t CORNER_PX        = 50;
    static constexpr uint32_t AUTO_DIM_MS      = 2UL * 60UL * 60UL * 1000UL;

    using BrightnessCallback = std::function<void(uint8_t)>;

    TouchDimmer(uint16_t screenW, uint16_t screenH, BrightnessCallback cb)
        : _w(screenW), _h(screenH), _setBrightness(cb),
          _lastActivityMs(millis()) {}

    // Call on every touch-down event (display-space coordinates).
    // Returns true if the event was consumed by the dimmer and should
    // not be passed to application touch handlers.
    bool onTouch(uint16_t x, uint16_t y) {
        uint32_t now = millis();
        _lastActivityMs = now;      // any touch resets inactivity timer

        if (_dimmed) {
            // Any tap on a dimmed screen just wakes — don't pass to app
            _wake();
            _firstTapMs = 0;        // wake tap can't be first of a double-tap
            return true;
        }

        bool inCorner = (x <= CORNER_PX) && (y >= (_h - CORNER_PX));

        if (inCorner) {
            if (_firstTapMs != 0 && (now - _firstTapMs) <= DOUBLE_TAP_MS) {
                _dim();
                _firstTapMs = 0;
                return true;        // consumed — don't pass double-tap to app
            }
            _firstTapMs = now;      // first tap: record, but don't consume
        } else {
            _firstTapMs = 0;        // out-of-corner tap cancels pending gesture
        }

        return false;
    }

    // Call every loop() iteration to handle auto-dim.
    void update() {
        if (!_dimmed && (millis() - _lastActivityMs) >= AUTO_DIM_MS) {
            _dim();
        }
    }

    bool isDimmed()     const { return _dimmed; }
    uint8_t brightness() const { return _dimmed ? BRIGHTNESS_DIM : BRIGHTNESS_FULL; }

private:
    void _dim()  { _dimmed = true;  _setBrightness(BRIGHTNESS_DIM);  }
    void _wake() { _dimmed = false; _setBrightness(BRIGHTNESS_FULL); }

    uint16_t           _w, _h;
    BrightnessCallback _setBrightness;
    bool               _dimmed       = false;
    uint32_t           _firstTapMs   = 0;
    uint32_t           _lastActivityMs;
};