// =============================
// File: config/Features.h
// =============================
#pragma once

// Feature flags (1=enable, 0=disable)
#define USE_SPS30       1 // U2, I2C
#define USE_BME688      1 // U3 , I2C
#define USE_CO_ADC      1 // U4 (CO analog via ADS1115 A0), GSET11-P110
#define USE_SMOKE2      1 // U5 (placeholder)
#define USE_ADS1115     1 // U6 (A0~A3), I2C
#define USE_ICS43434    0 // U7 (I2S Mic)
#define USE_SGP30 		1 // U8 (TVOC/eCO2), I2C
#define USE_SEN0177 	0 // J1 PM UART
#define USE_ZE07 		1 // J2 ZE07-CO UART
#define USE_MQ2 		1 // J12 MQ-2 via ADS1115 A1
