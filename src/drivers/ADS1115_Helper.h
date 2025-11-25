// =============================
// File: drivers/ADS1115_Helper.h
// =============================
#pragma once
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>


class ADS1115_Helper {
    public:
        bool begin(uint8_t addr=0x48);
        float read_mV(uint8_t ch); // accounting your x2 divider on inputs
        float read_V(uint8_t ch);
        int16_t read_raw(uint8_t ch);
        Adafruit_ADS1115& dev(){ return _ads; }

    private:
        Adafruit_ADS1115 _ads;
};