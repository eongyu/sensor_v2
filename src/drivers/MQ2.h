// =============================
// File: drivers/MQ2.h
// =============================
#pragma once
#include <Arduino.h>


struct MQ2Config {
	float v_div_ratio = 0.625f; // ADC / AO
	float rl_ohms = 5000.0f; // load resistor
	float v_supply_mV = 5000.0f; // sensor supply
	uint32_t warmup_s = 15; // heater warmup
	uint32_t calib_s = 20; // R0 averaging
	float ema_alpha = 0.2f; // smoothing
	float alarm_thr = 0.35f; // Rs/R0 alarm threshold
};


class MQ2 {
	public:
		enum Phase { WARMUP, CALIB, RUN };
		MQ2(){};
		void begin(const MQ2Config &cfg, float r0_value = 0.0f);
		// feed AO voltage seen *at ADC input* (mV)
		void update_from_adc_mV(float adc_mV);
		// getters
		Phase phase() const { return _ph; }
		float r0() const { return _r0; }
		float rs() const { return _rs; }
		float ratio() const { return _ratio; }
		float ratio_ema() const { return _ema; }
		bool alarm() const { return _alarm; }
		float calc_Rs_from_AO_mV_(float v_adc_mV) const; // using divider, RL, Vs
	private:
		MQ2Config _cfg; Phase _ph=WARMUP; uint32_t _t0=0;
		float _r0=NAN, _rs=NAN, _ratio=NAN, _ema=NAN; bool _alarm=false; uint32_t _count=0;
};