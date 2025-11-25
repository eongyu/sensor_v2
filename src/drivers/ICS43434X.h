// =============================
// File: drivers/ICS43434X.h
// =============================
#pragma once
#include <Arduino.h>
#include <driver/i2s.h>


class ICS43434X {
	public:
		bool begin(int bclk, int lrck, int din, int fs=16000);
		float read_rms();
	private:
		i2s_port_t _port=I2S_NUM_0; size_t _n=1024;
};