// =============================
// File: drivers/SGP30X.h
// =============================
#pragma once
#include <Arduino.h>
#include <Adafruit_SGP30.h>


class SGP30X {
	public:
		bool begin();
		bool read(uint16_t &eco2, uint16_t &tvoc);
	private:
		Adafruit_SGP30 _sgp;
};