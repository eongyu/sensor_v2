// =============================
// File: drivers/ZE07.h
// =============================
#pragma once
#include <Arduino.h>

struct PM25Data { uint16_t pm1_0, pm2_5, pm10; };

class SEN0177 {
	public:
		bool begin(HardwareSerial &ser, int rxPin, int txPin, uint32_t baud=9600);
		bool read(PM25Data &out);
	private:
		HardwareSerial *_ser=nullptr;
};
