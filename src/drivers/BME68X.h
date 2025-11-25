// =============================
// File: drivers/BME688X.h
// =============================
#pragma once
#include <Arduino.h>
#include <Adafruit_BME680.h>


class BME68X {
	public:
		bool begin(uint8_t addr=0x76);
		bool read(float &tempC, float &hum, float &gas_ohm);
		float last_gas_resistance() const { return _last_gas_res; }
	private:
		Adafruit_BME680 _bme;
		float _last_gas_res = -1.0f; // 마지막 가스 저항 값 저장, 초기값 -1
};