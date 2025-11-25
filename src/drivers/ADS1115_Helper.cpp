// =============================
// File: drivers/ADS1115_Helper.cpp
// =============================
#include "ADS1115_Helper.h"

bool ADS1115_Helper::begin(uint8_t addr){
    if(!_ads.begin(addr)) return false;
    _ads.setGain(GAIN_TWOTHIRDS); // ±6.144V → 0.1875mV/LSB

    return true;
}

int16_t ADS1115_Helper::read_raw(uint8_t ch) {
    return _ads.readADC_SingleEnded(ch);
}

float ADS1115_Helper::read_mV(uint8_t ch) {
    int16_t raw = read_raw(ch);
    return 2.0f * raw * 0.1875f; // x2 for your external divider
}

float ADS1115_Helper::read_V(uint8_t ch) { 
    return read_mV(ch)/1000.0f; 
}