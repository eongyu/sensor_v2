// =============================
// File: drivers/MQ2.cpp
// =============================
#include "MQ2.h"


void MQ2::begin(const MQ2Config &cfg, float r0_value){
	_cfg=cfg; _t0=millis(); _ema=NAN; _alarm=false; _count=0;

	// 외부에서 유효한 R0 값을 제공하면, 교정 단계를 건너뛰고 바로 RUN 상태로 시작
	if (r0_value > 0.0f) {
		_r0 = r0_value;
		_ph = RUN;
	} else {
		_ph = WARMUP; // R0 값이 없으면 기존처럼 자동 교정 시작
		_r0 = NAN;
	}
}

float MQ2::calc_Rs_from_AO_mV_(float v_adc_mV) const {
	float v_ao = v_adc_mV / _cfg.v_div_ratio; // restore AO
	if(v_ao < 1.0f) v_ao = 1.0f; // protect div0
	float Vc = _cfg.v_supply_mV;

	return _cfg.rl_ohms * (Vc - v_ao) / v_ao; // Rs = RL*(Vc-Vo)/Vo
}


void MQ2::update_from_adc_mV(float adc_mV){
	uint32_t now = millis();
	_rs = calc_Rs_from_AO_mV_(adc_mV);


	switch(_ph){
		case WARMUP:
			if(now - _t0 >= _cfg.warmup_s*1000UL){ _ph=CALIB; _t0=now; _r0=0.0f; _count=0; }
			break;
		case CALIB:
			_r0 += _rs; _count++;
			if(now - _t0 >= _cfg.calib_s*1000UL){
			if(_count>0) _r0 /= (float)_count; if(_r0 < 100.0f) _r0 = 100.0f; _ph = RUN; _ema = 1.0f; _t0=now;
			}
			break;
		case RUN:
			break;
	}


	if(!isnan(_r0) && _r0>0){
		_ratio = _rs/_r0;
		if(isnan(_ema)) _ema=_ratio;
		_ema = (1.0f-_cfg.ema_alpha)*_ema + _cfg.ema_alpha*_ratio;
		_alarm = (_ph==RUN) && (_ema < _cfg.alarm_thr);
	}
}