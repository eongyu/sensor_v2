// =============================
// File: config/BuildOpts.h
// =============================
#pragma once

// ADC resolution for ESP32-S3
#ifndef ADC_RES_BITS
    #define ADC_RES_BITS 12
#endif


// I2C speed - 400kHz
#ifndef I2C_FREQ_HZ
    //#define I2C_FREQ_HZ 400000
    #define I2C_FREQ_HZ 100000          // SPS30 은 100KHz 동작
#endif
