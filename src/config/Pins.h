// =============================
// File: config/Pins.h
// =============================
#pragma once
#include <Arduino.h>


// LEDs
#define PIN_LED_RING    48
#define PIN_LED1        19
#define PIN_LED2        20
#define PIN_LED3 		21
#define PIN_LED4 		14


// I2C
#define PIN_I2C_SDA 	8
#define PIN_I2C_SCL 	9


// I2S (ICS-43434)
#define PIN_I2S_BCLK 	14
#define PIN_I2S_LRCK 	15
#define PIN_I2S_DIN 	16


// UARTs
#define PM_TX_PIN 		17 // UART1 TX
#define PM_RX_PIN 		18 // UART1 RX

#define CO_TX_PIN 		15 // UART2 TX
#define CO_RX_PIN 		16 // UART2 RX