// =============================
// File: drivers/SMOKE2.cpp
// =============================
#include "SMOKE2.h"

// ===== Low-level I2C =====
bool SMOKE2::writeReg16(uint8_t reg, uint16_t val){
	_w->beginTransmission(_addr);
	_w->write(reg);
	_w->write(uint8_t(val>>8));
	_w->write(uint8_t(val&0xFF));
	return _w->endTransmission()==0;
}

bool SMOKE2::readReg16(uint8_t reg, uint16_t &out){
	_w->beginTransmission(_addr);
	_w->write(reg);
	if(_w->endTransmission(false)!=0) return false;             // repeated start
	if(_w->requestFrom(int(_addr),2)!=2) return false;
	uint8_t msb=_w->read(), lsb=_w->read();
	out=(uint16_t(msb)<<8)|lsb;
	return true;
}

// 8바이트(4워드) 버스트 읽기: stop true/false 모두 시도 후 폴백
bool SMOKE2::readFIFOWords(uint16_t* buf, size_t words){
	const uint8_t need = words * 2;

	// try #1: sendStop=false
	_w->beginTransmission(_addr);
	_w->write(REG_FIFO_DATA);
	if (_w->endTransmission(false) == 0) {
		int got = _w->requestFrom((uint8_t)_addr, need, (uint8_t)false);
		if (got == need) {
		for (size_t i=0;i<words;++i){
			int msb=_w->read(), lsb=_w->read();
			if (msb<0 || lsb<0) return false;
			buf[i] = (uint16_t(uint8_t(msb))<<8) | uint8_t(lsb);
		}
		return true;
		}
	}

	// try #2: sendStop=true
	_w->beginTransmission(_addr);
	_w->write(REG_FIFO_DATA);
	if (_w->endTransmission(false) == 0) {
		int got = _w->requestFrom((uint8_t)_addr, need, (uint8_t)true);
		if (got == need) {
		for (size_t i=0;i<words;++i){
			int msb=_w->read(), lsb=_w->read();
			if (msb<0 || lsb<0) return false;
			buf[i] = (uint16_t(uint8_t(msb))<<8) | uint8_t(lsb);
		}
		return true;
		}
	}

	// try #3: 16비트씩 폴백
	for(size_t i=0;i<words;++i){
		if(!readReg16(REG_FIFO_DATA, buf[i])) return false;
	}
	return true;
}

// ===== eFuse (AN-2033) =====
bool SMOKE2::efuse_enable(){
	uint16_t v=0;
	readReg16(REG_CLK32K, v); writeReg16(REG_CLK32K, v | (1u<<7)); delay(2);
	writeReg16(REG_MODE, 0x0001); delay(2); // PROGRAM
	writeReg16(REG_CLK32M_EN, 0x0001);
	writeReg16(REG_EFUSE_CTRL, 0x0007);
	// wait refresh complete (0x67 LSB==0x04)
	uint16_t st=0; uint32_t t0=millis();
	do { readReg16(REG_EFUSE_STS, st); } while( ( (st & 0x00FF) != 0x04 ) && (millis()-t0<50) );
	return (st & 0x00FF)==0x04;
}
void SMOKE2::efuse_disable(){
	writeReg16(REG_EFUSE_CTRL, 0x0000);
	writeReg16(REG_CLK32M_EN,  0x0000);
}
bool SMOKE2::read_efuse_cal(uint8_t& mod_id, uint8_t& l1_gain, uint8_t& l3_gain, uint8_t& l1_int, uint8_t& l3_int){
	if(!efuse_enable()) return false;
	uint16_t v=0;
	if(!readReg16(REG_MODULE_ID, v)){ efuse_disable(); return false; }
	mod_id = v & 0xFF;
	readReg16(REG_L1_GAIN, v); l1_gain = v & 0xFF;
	readReg16(REG_L3_GAIN, v); l3_gain = v & 0xFF;
	readReg16(REG_L1_INT,  v); l1_int  = v & 0xFF;
	readReg16(REG_L3_INT,  v); l3_int  = v & 0xFF;
	efuse_disable();
	return true;
}
void SMOKE2::compute_gaincal(uint8_t mod_id, uint8_t l1_gain, uint8_t l3_gain, uint8_t l1_int, uint8_t l3_int){
	auto calc = [&](bool blue)->float{
		float LEDmA = blue ? _led1_mA : _led3_mA;
		float gc = blue ? l1_gain : l3_gain;
		float ic = blue ? l1_int  : l3_int;
		float x_gain, x_int, x_mean_gain, x_mean_int;

		if (mod_id==30 || mod_id==31){
		x_gain = ( (blue?17.f:34.f)/256.f ) * (gc - 112.f) + (blue?17.f:34.f);
		x_int  = (blue?8.f:5.f) * (ic - 128.f);
		x_mean_gain = blue?17.f:34.f; x_mean_int = blue?622.f:128.f;
		} else { // 33 계열
		x_gain = ( (blue?21.f:42.f)/256.f ) * (gc - 112.f) + (blue?21.f:42.f);
		x_int  = (blue?8.f:5.f) * (ic - 80.f);
		x_mean_gain = blue?21.f:42.f; x_mean_int = blue?753.f:156.f;
		}
		float device_scalar  = x_gain*LEDmA + x_int;
		float nominal_scalar = x_mean_gain*LEDmA + x_mean_int;
		if (nominal_scalar <= 0.0f) return 1.f;
		return device_scalar / nominal_scalar;
	};
	_gcal_blue = calc(true);
	_gcal_ir   = calc(false);
	_cal_ready = true;
}

// ===== ADPD188 init =====
void SMOKE2::adpd_init(){
	// 32k enable
	uint16_t clk=0; readReg16(REG_CLK32K, clk); writeReg16(REG_CLK32K, clk | (1u<<7)); delay(2);
	// PROGRAM
	writeReg16(REG_MODE, 0x0001); delay(2);

	// FIFO: A/B 32-bit sum → FIFO
	writeReg16(REG_FIFO_CFG,    0x30A9);
	writeReg16(REG_SAMPLE_RATE, 0x0200); // ~16Hz
	writeReg16(REG_SLOT_SEL,    0x011D); // Blue->A, IR->B, PDs combine
	writeReg16(REG_DECIM,       0x0000);

	// 권장: chop/maths ON (필요시 raw 확인하려면 0으로)
	writeReg16(REG_SLOTA_CHOP,  0x0009);
	writeReg16(REG_SLOTA_OFST,  0x0000);
	writeReg16(REG_SLOTB_CHOP,  0x0009);
	writeReg16(REG_SLOTB_OFST,  0x0000);

	// LED/TIA/적분시간
	writeReg16(REG_LED3_DRV, _reg_led3_drv); // IR
	writeReg16(REG_LED1_DRV, _reg_led1_drv); // BLUE
	writeReg16(REG_LED2_DRV, 0x1530);
	writeReg16(REG_LED_TRIM, 0x630C);

	writeReg16(REG_SLOTA_TIM0, 0x0320);
	writeReg16(REG_SLOTA_TIM1, 0x040E);
	writeReg16(REG_SLOTB_TIM0, 0x0320);
	writeReg16(REG_SLOTB_TIM1, 0x040E);

	writeReg16(REG_INTEG_A, _reg_integ_a);
	writeReg16(REG_INTEG_B, _reg_integ_b);
	writeReg16(REG_PWRDOWN, 0x31C6);
	writeReg16(REG_TIA_A,   _reg_tia_a);
	writeReg16(REG_PATH_A,  0xADA5);
	writeReg16(REG_TIA_B,   _reg_tia_b);
	writeReg16(REG_PATH_B,  0xADA5);

	writeReg16(REG_CHOP_MATH, 0x0544);     // ★ 권장값
	writeReg16(REG_PD_BIAS,   0x0AA0);     // ★ 필수 (PD 바이어스)



	// eFuse 보정(옵션)
	if(_use_efuse_cal){
		uint8_t id=0,g1=0,g3=0,i1=0,i3=0;
		if(read_efuse_cal(id,g1,g3,i1,i3)){
		compute_gaincal(id,g1,g3,i1,i3);
		}
	}
		// NORMAL (샘플링 시작)
	writeReg16(REG_MODE, 0x0002);
	delay(5);
}

// 이전 버전 호환성을 위한 begin 함수
bool SMOKE2::begin(TwoWire &w, uint8_t i2c_addr){
	return begin(w, i2c_addr, 0.0f); // precal_alpha를 0.0f로 하여 새로운 begin 함수 호출
}

bool SMOKE2::begin(TwoWire &w, uint8_t i2c_addr, float precal_alpha){
	_w=&w; _addr=i2c_addr;
	_cal_ready=false; _endian_fixed=false; _use_ba=true;

	if (precal_alpha > 0.0f) {
		_ema_ratio = precal_alpha;
		_baseline_ready = true;
		_samples_seen = _warmup_samples; // 예열 단계를 건너뛰었음을 표시
	} else {
		_samples_seen=0; _baseline_ready=false; _ema_ratio=0.f;
	}
	adpd_init();
	delay(10);
	#if SMOKE2_DEBUG
		debug_after_init(Serial);
	#endif
	return true;
}

// ===== Read & process =====
bool SMOKE2::read(Reading &out){
	// FIFO 상태
	uint16_t status=0;
	if(!readReg16(REG_STATUS_FIFO,status)) return false;
	uint8_t fifo_bytes=(status>>8)&0xFF;
	const uint8_t PACKET_BYTES=8;
	if(fifo_bytes<PACKET_BYTES) return false;

	// 여러 패킷 평균
	uint8_t can=fifo_bytes/PACKET_BYTES;
	uint8_t navg = can < _packets_to_avg ? can : _packets_to_avg;
	if(navg==0) navg=1;

	uint64_t accB=0, accIR=0;
	uint8_t nread = 0;

	for(uint8_t i=0;i<navg;i++){
		uint16_t w[4];
		if(!readFIFOWords(w,4)) break;

		if(!_endian_fixed){
		uint32_t Aab=(uint32_t(w[0])<<16)|w[1], Aba=(uint32_t(w[1])<<16)|w[0];
		uint32_t Bab=(uint32_t(w[2])<<16)|w[3], Bba=(uint32_t(w[3])<<16)|w[2];
		auto ok=[](uint32_t v){ return v>100 && v<200000000U; };
		bool ba_ok = ok(Aba)&&ok(Bba);
		bool ab_ok = ok(Aab)&&ok(Bab);
		_use_ba = ba_ok || (!ab_ok); // BA가 타당하면 BA, 아니면 AB
		_endian_fixed = true;
		#if SMOKE2_DEBUG
		Serial.printf("[SMOKE2] endian=%s (Aab=%u Aba=%u | Bab=%u Bba=%u)\n",
						_use_ba?"BA":"AB", Aab,Aba,Bab,Bba);
		#endif
		}

		uint32_t blue = _use_ba ? ((uint32_t(w[1])<<16)|w[0]) : ((uint32_t(w[0])<<16)|w[1]);
		uint32_t ir   = _use_ba ? ((uint32_t(w[3])<<16)|w[2]) : ((uint32_t(w[2])<<16)|w[3]);

		accB += blue;
		accIR += ir;
		nread++;
	}
	if(nread==0) return false;

	uint32_t blue = uint32_t(accB / nread);
	uint32_t ir   = uint32_t(accIR / nread);

	out.raw_blue = blue;
	out.raw_ir   = ir;
	out.navg     = nread;

	// === eFuse 보정 정규화 (선택) ===
	float blue_n = _cal_ready ? (float(blue) / _gcal_blue) : float(blue);
	float ir_n   = _cal_ready ? (float(ir)   / _gcal_ir)   : float(ir);

	// ratio/alpha
	float ratio_now = (ir_n>0) ? (blue_n/ir_n) : 0.0f;
	out.alpha_updated = false;

	if(_samples_seen < _warmup_samples){
		_samples_seen++;
		_ema_ratio = (_samples_seen==1) ? ratio_now
										: (1.f-_ema_alpha)*_ema_ratio + _ema_alpha*ratio_now;
		out.alpha_updated = true;
		if(_samples_seen>=_warmup_samples) _baseline_ready=true;
	} else {
		if(!_alpha_locked){
		float delta=fabsf(ratio_now-_ema_ratio);
		if(delta < _adapt_guard){ // 이벤트 중에는 업데이트 정지 효과
			_ema_ratio = (1.f-_ema_alpha)*_ema_ratio + _ema_alpha*ratio_now;
			out.alpha_updated = true;
		}
		}
	}

	float alpha = (_ema_ratio>0.f)? _ema_ratio : 0.02f;

	// score = ±(ratio - alpha) * IR_eff  (투과형이면 부호 반전)
	// IR 값으로 스케일링하여 score의 범위를 안정화시킵니다.
	float ir_eff = (ir_n>0.f)? ir_n : 0.f;
	if(ir_eff < _min_ir_for_scaling) ir_eff=_min_ir_for_scaling;

	// IR 값의 변동에 따른 score 증폭을 막기 위해, score 계산에서 IR 값의 영향을 제거합니다.
	// 오직 (ratio - alpha)의 차이에만 의존하도록 하고, 큰 상수를 곱해 정수 범위로 만듭니다.
	float core = (ratio_now - alpha) * 1000000.0f;
	float score = _transmissive ? -core : core;

	// 퍼시스턴스 + 히스테리시스(OFF 쪽 느리게)
	bool on_now = _baseline_ready && (score > _th_score);
	if(on_now){ _cnt_on++; _cnt_off=0; } else { _cnt_off++; _cnt_on=0; }
	if(!_alarm_state && _cnt_on  >= _persist_on)  _alarm_state=true;
	if( _alarm_state && _cnt_off >= _persist_off) _alarm_state=false;

	// 결과
	out.blue=blue; out.ir=ir; out.ratio=ratio_now; out.alpha=alpha;
	out.score=score; out.alarm=_alarm_state; out.ir_eff=ir_eff;

	#if SMOKE2_DEBUG
	static uint32_t lastLog=0; uint32_t now=millis();
	if(now-lastLog>500){
		lastLog=now;
		Serial.printf("[SMOKE2] B=%u IR=%u r=%.5f a=%.5f s=%.0f navg=%u upd=%d alarm=%d\n",
					blue, ir, ratio_now, alpha, score, out.navg, out.alpha_updated, out.alarm);
	}
	#endif

	return true;
}

// ===== Diagnostics =====
bool SMOKE2::checkMapping(){
	uint16_t v=0; if(!readReg16(REG_SLOT_SEL,v)) return false;
	return v==0x011D;
}
void SMOKE2::dumpKeyRegs(Stream& s){
	const uint8_t regs[]={
		REG_MODE,REG_FIFO_CFG,REG_SAMPLE_RATE,REG_SLOT_SEL,REG_DECIM,
		REG_LED1_DRV,REG_LED3_DRV,REG_TIA_A,REG_TIA_B,REG_INTEG_A,REG_INTEG_B,REG_PD_BIAS
	};
	for(uint8_t r:regs){
		uint16_t v=0; if(readReg16(r,v)) { s.print("R[0x"); s.print(r,HEX); s.print("]=0x"); s.println(v,HEX); }
	}
}
void SMOKE2::debug_after_init(Stream& s){
	uint16_t v;
	readReg16(REG_MODE, v);        s.printf("MODE=0x%04X\n", v);
	readReg16(REG_SLOT_SEL, v);    s.printf("SLOT_SEL=0x%04X\n", v);
	readReg16(REG_FIFO_CFG, v);    s.printf("FIFO_CFG=0x%04X\n", v);
	readReg16(REG_SAMPLE_RATE, v); s.printf("SAMPLE_RATE=0x%04X\n", v);
	readReg16(REG_TIA_A, v);       s.printf("TIA_A=0x%04X\n", v);
	readReg16(REG_TIA_B, v);       s.printf("TIA_B=0x%04X\n", v);
	readReg16(REG_LED1_DRV, v);    s.printf("LED1(BLUE)=0x%04X\n", v);
	readReg16(REG_LED3_DRV, v);    s.printf("LED3(IR)=0x%04X\n", v);
	readReg16(REG_PD_BIAS, v);     s.printf("PD_BIAS=0x%04X\n", v);
	if(readReg16(REG_STATUS_FIFO, v)){
		uint8_t bytes=(v>>8)&0xFF; s.printf("STATUS_FIFO=0x%04X bytes=%u\n", v, bytes);
	} else s.println("STATUS_FIFO read FAIL");
}

void SMOKE2::debug_fifo_probe(Stream& s){
	uint16_t s1=0, s2=0;
	if(!readReg16(REG_STATUS_FIFO, s1)){ s.println("[SMOKE2] STATUS read FAIL"); return; }
	uint8_t b1=(s1>>8)&0xFF;
	s.printf("[SMOKE2] FIFO bytes before=%u\n", b1);

	uint16_t w[4];
	bool ok = readFIFOWords(w,4);
	s.printf("[SMOKE2] readFIFOWords ok=%d\n", ok);

	if(!readReg16(REG_STATUS_FIFO, s2)){ s.println("[SMOKE2] STATUS read FAIL (after)"); return; }
	uint8_t b2=(s2>>8)&0xFF;
	s.printf("[SMOKE2] FIFO bytes %u -> %u\n", b1, b2);

	if(ok){
		s.printf("[SMOKE2] w: %04X %04X %04X %04X\n", w[0], w[1], w[2], w[3]);
		uint32_t Aab=(uint32_t(w[0])<<16)|w[1];
		uint32_t Aba=(uint32_t(w[1])<<16)|w[0];
		uint32_t Bab=(uint32_t(w[2])<<16)|w[3];
		uint32_t Bba=(uint32_t(w[3])<<16)|w[2];
		auto ok32=[](uint32_t v){ return v>100 && v<200000000U; };
		bool use_ba = (ok32(Aba)&&ok32(Bba)) || !(ok32(Aab)&&ok32(Bab));
		uint32_t BLUE = use_ba ? Aba : Aab;
		uint32_t IR   = use_ba ? Bba : Bab;
		s.printf("[SMOKE2] Aab=%u Aba=%u | Bab=%u Bba=%u | pick=%s | BLUE=%u IR=%u ratio=%.5f\n",
				Aab, Aba, Bab, Bba, use_ba?"BA":"AB", BLUE, IR, IR? (double)BLUE/IR:0.0);
	}
}

// 내부 데이터 레지스터 덤프(참고용)
void SMOKE2::smoke2_dump_data_regs(SMOKE2 &dev, Stream& s){
	auto rd = [&](uint8_t reg)->uint16_t{
		uint16_t v=0; if(!dev.readReg16(reg, v)) { s.printf("R[0x%02X] FAIL\n", reg); }
		return v;
	};
	uint16_t A16_0 = rd(0x64), A16_1 = rd(0x65), A16_2 = rd(0x66), A16_3 = rd(0x67);
	uint16_t B16_0 = rd(0x68), B16_1 = rd(0x69), B16_2 = rd(0x6A), B16_3 = rd(0x6B);
	uint16_t A27_0L = rd(0x70), A27_0H = rd(0x71);
	uint16_t B27_0L = rd(0x78), B27_0H = rd(0x79);

	s.printf("[DATA16] A: %04X %04X %04X %04X | B: %04X %04X %04X %04X\n",
			A16_0, A16_1, A16_2, A16_3, B16_0, B16_1, B16_2, B16_3);
	s.printf("[DATA27] A0: L=%04X H=%04X | B0: L=%04X H=%04X\n",
			A27_0L, A27_0H, B27_0L, B27_0H);
}
