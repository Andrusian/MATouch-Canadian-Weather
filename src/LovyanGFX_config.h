#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// ----------------------------------------------------------------
// MaTouch ESP32-S3 3.5" ILI9488 - pin assignments from Makerfabs
// SPI_9488.h (known good). Touch pins TBD - not in their demo.
// ----------------------------------------------------------------

#define LCD_MOSI 13
#define LCD_MISO 12
#define LCD_SCK  14
#define LCD_CS   15
#define LCD_RST  -1
#define LCD_DC   21
#define LCD_BL   48

class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9488  _panel_instance;
    lgfx::Bus_SPI        _bus_instance;
    lgfx::Light_PWM      _light_instance;
    lgfx::Touch_FT5x06   _touch_instance;  // FT6236 is FT5x06 family

public:
    LGFX(void) {
        // --- SPI bus ---
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host   = SPI3_HOST;
            cfg.spi_mode   = 0;
            cfg.freq_write = 40000000;
            cfg.freq_read  = 16000000;
            cfg.spi_3wire  = true;
            cfg.use_lock   = true;
            cfg.dma_channel = 1;
            cfg.pin_sclk   = LCD_SCK;
            cfg.pin_mosi   = LCD_MOSI;
            cfg.pin_miso   = LCD_MISO;
            cfg.pin_dc     = LCD_DC;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        // --- Display panel ---
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs           = LCD_CS;
            cfg.pin_rst          = LCD_RST;
            cfg.pin_busy         = -1;
            cfg.memory_width     = 320;
            cfg.memory_height    = 480;
            cfg.panel_width      = 320;
            cfg.panel_height     = 480;
            cfg.offset_x         = 0;
            cfg.offset_y         = 0;
            cfg.offset_rotation  = 1;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits  = 1;
            cfg.readable         = true;
            cfg.invert           = false;
            cfg.rgb_order        = false;
            cfg.dlen_16bit       = false;
            cfg.bus_shared       = true;
            _panel_instance.config(cfg);
        }

        // --- Backlight (GPIO 48, PWM) ---
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl      = LCD_BL;
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        // --- Touch (FT6236 via I2C) ---
        // Pins not provided in Makerfabs demo - these are educated guesses.
        // If touch doesn't work, check schematic for actual SDA/SCL/INT pins.
        {
            auto cfg = _touch_instance.config();
            cfg.i2c_port = 0;
            cfg.i2c_addr = 0x38;
            cfg.pin_sda  = 38;
            cfg.pin_scl  = 39;
            cfg.pin_int  = 40;
            cfg.pin_rst  = -1;
            cfg.freq     = 400000;
            cfg.x_min    = 0;
            cfg.x_max    = 479;
            cfg.y_min    = 0;
            cfg.y_max    = 319;
            cfg.offset_rotation = 3;
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }
};