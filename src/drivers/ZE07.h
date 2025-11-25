// =============================
// File: drivers/ZE07.h
// =============================
#pragma once
#include <Arduino.h>


class ZE07 {
	public:
		bool begin(HardwareSerial &ser, int rxPin, int txPin, uint32_t baud=9600);
		bool read_frame(uint32_t timeout_ms=200);
		bool parse_ppm(float &ppm, uint16_t &full_range_ppm, uint8_t &decimals) const;
		void setQA(bool qa_mode);
	private:
		HardwareSerial *_ser=nullptr; uint8_t _frame[9]{}; bool _qa=false;
		static uint8_t checksum_(const uint8_t *buf){ uint8_t s=0; for(int i=1;i<=7;++i) s+=buf[i]; return (uint8_t)((~s)+1);}
};