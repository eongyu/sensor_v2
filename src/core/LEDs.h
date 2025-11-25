// =============================
// File: core/LEDs.h
// =============================
#pragma once
#include <Arduino.h>
#include "../config/Pins.h"


namespace leds {
    void init();
    void blink1(bool on);
    void blink2(bool on);
    void blink3(bool on);
    void blink4(bool on);
    
    void set1(bool on);
    void set2(bool on);
    void set3(bool on);
    void set4(bool on);
}