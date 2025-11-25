// =============================
// File: drivers/SMOKE2.h
// =============================
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define SMOKE2_DEBUG 0   // 초기화/읽기 진단 로그 활성화

class SMOKE2 {
public:
  struct Reading {
    uint32_t blue=0, ir=0;        // 평균된 원시 합계(정규화 전)
    float    ratio=0, alpha=0, score=0;
    bool     alarm=false;
    uint8_t  navg=0;              // 이번에 평균된 패킷 수
    bool     alpha_updated=false; // 이번 사이클에 alpha 갱신 여부
    float    ir_eff=0;            // score 계산에 사용된 IR 하한 보정값
    uint32_t raw_blue=0, raw_ir=0;// 내부 확인용
  };

  // ===== 설정 세터 =====
  void setAddr(uint8_t a=0x64){ _addr=a; }
  void setSampleHz(uint16_t hz=16){ _sample_hz=hz; _warmup_samples = _sample_hz * _warmup_sec; }
  void setWarmupSec(uint16_t s=15){ _warmup_sec=s; _warmup_samples = _sample_hz * _warmup_sec; }
  void setEmaAlpha(float a=0.01f){ _ema_alpha=a; }
  void setThreshold(float th=1.0e5f){ _th_score=th; }
  void setPacketsToAvg(uint8_t n){ _packets_to_avg = n<1?1:(n>8?8:n); }
  void setAdaptGuard(float g=0.02f){ _adapt_guard=g; }     // ratio 변화 2% 이내만 alpha 갱신
  void freezeAlpha(bool on){ _alpha_locked=on; }
  void setMinIrForScaling(float v=2000.f){ _min_ir_for_scaling=v; }
  void setPersist(uint16_t onN=3, uint16_t offN=5){ _persist_on=onN; _persist_off=offN; }
  float getAlpha() const { return _ema_ratio; }

  bool isBaselineReady() const { return _baseline_ready; }
  // LED/TIA/적분시간 튜닝값(필요 시 begin() 전에 호출)
  void setLedBlueReg(uint16_t v){ _reg_led1_drv=v; }
  void setLedIrReg(uint16_t v){ _reg_led3_drv=v; }
  void setTiaA(uint16_t v){ _reg_tia_a=v; }
  void setTiaB(uint16_t v){ _reg_tia_b=v; }
  void setIntegA(uint16_t v){ _reg_integ_a=v; }
  void setIntegB(uint16_t v){ _reg_integ_b=v; }

  // eFuse 보정(AN-2033) — LED 전류(mA) 지정 & 사용 토글
  void setLedCurrents_mA(float blue_mA, float ir_mA){ _led1_mA=blue_mA; _led3_mA=ir_mA; }
  void enableEfuseCalibration(bool on){ _use_efuse_cal=on; }

  // 투과형 배치(감쇠 ↑)일 때 score 부호 반전 옵션
  void setTransmissiveMode(bool on){ _transmissive = on; }

  // ===== 수명주기 =====
  bool begin(TwoWire &w, uint8_t i2c_addr); // 이전 버전 호환성을 위한 선언 추가
  bool begin(TwoWire &w=Wire, uint8_t i2c_addr=0x64, float precal_alpha = 0.0f);
  bool read(Reading &out);

  // 진단
  bool checkMapping();           // Blue->A, IR->B ?
  void dumpKeyRegs(Stream& s);   // 주요 레지스터 값 출력
  void debug_fifo_probe(Stream& s);
  void smoke2_dump_data_regs(SMOKE2 &dev, Stream& s);

private:
  // ===== 내부 상태 =====
  TwoWire* _w=nullptr;
  uint8_t  _addr=0x64;
  uint16_t _sample_hz=16;
  uint16_t _warmup_sec=15;
  float    _ema_alpha=0.01f;
  uint16_t _warmup_samples=16*15;
  float    _th_score=1.0e5f;

  uint8_t  _packets_to_avg=4;
  float    _adapt_guard=0.02f;
  bool     _alpha_locked=false;
  float    _min_ir_for_scaling=2000.f;
  uint16_t _persist_on=3,_persist_off=5;
  bool     _alarm_state=false;
  uint16_t _cnt_on=0,_cnt_off=0;

  uint32_t _samples_seen=0;
  bool     _baseline_ready=false;
  float    _ema_ratio=0.f;

  // 레지스터 튜닝 값(기본: 권장치)
  uint16_t _reg_led1_drv=0x3536; // BLUE
  uint16_t _reg_led3_drv=0x3539; // IR
  uint16_t _reg_tia_a   =0x1C34; // 200k (필요 시 0x1C36=1M)
  uint16_t _reg_tia_b   =0x1C34; // 200k
  uint16_t _reg_integ_a =0x22F0;
  uint16_t _reg_integ_b =0x22F0;

  // eFuse 보정
  bool   _use_efuse_cal = true;
  bool   _cal_ready = false;
  float  _led1_mA = 20.f;  // 실제 전류에 맞게 setLedCurrents_mA()로 지정 권장
  float  _led3_mA = 20.f;
  float  _gcal_blue = 1.f, _gcal_ir = 1.f;

  // 엔디안 자동 판별(최초 1회)
  bool   _endian_fixed = false;
  bool   _use_ba = true;   // true: (w[1]<<16)|w[0] / false: (w[0]<<16)|w[1]

  // 투과형 모드(감쇠↑)일 때 score 부호 반전
  bool   _transmissive = false;

  // ===== I2C helpers =====
  bool writeReg16(uint8_t reg, uint16_t val);
  bool readReg16(uint8_t reg, uint16_t &out);
  bool readFIFOWords(uint16_t* buf, size_t words); // burst read

  // ===== 칩 초기화 & eFuse =====
  void adpd_init();
  bool efuse_enable();
  void efuse_disable();
  bool read_efuse_cal(uint8_t& mod_id, uint8_t& l1_gain, uint8_t& l3_gain, uint8_t& l1_int, uint8_t& l3_int);
  void compute_gaincal(uint8_t mod_id, uint8_t l1_gain, uint8_t l3_gain, uint8_t l1_int, uint8_t l3_int);

  // ===== 레지스터 맵 (데이터시트 일치) =====
  enum : uint8_t {
    REG_STATUS_FIFO=0x00,  // [15:8] FIFO bytes
	REG_DEVID=0x08,
	REG_SW_RESET=0x0F,
    REG_MODE=0x10,
    REG_FIFO_CFG=0x11,
    REG_SAMPLE_RATE=0x12,
    REG_SLOT_SEL=0x14,
    REG_DECIM=0x15,
    REG_SLOTA_CHOP=0x17, 
	REG_SLOTA_OFST=0x18,
    REG_SLOTB_CHOP=0x1D, 
	REG_SLOTB_OFST=0x1E,

    REG_LED3_DRV=0x22, 
	REG_LED1_DRV=0x23, 
	REG_LED2_DRV=0x24, 
	REG_LED_TRIM=0x25,

    REG_SLOTA_TIM0=0x30, 
	REG_SLOTA_TIM1=0x31,
    REG_SLOTB_TIM0=0x35, 
	REG_SLOTB_TIM1=0x36,

    REG_INTEG_A=0x39, 
	REG_INTEG_B=0x3B, 
	REG_PWRDOWN=0x3C,
    REG_TIA_A=0x42,   
	REG_PATH_A=0x43,
    REG_TIA_B=0x44,  
	REG_PATH_B=0x45,

    REG_PD_BIAS=0x54,
    REG_CLK32K=0x4B, 
	REG_CHOP_MATH=0x58,

    // eFuse 관련 (AN-2033)
    REG_EFUSE_CTRL=0x57, 
	REG_CLK32M_EN=0x5F, 
	REG_EFUSE_STS=0x67,
    REG_MODULE_ID=0x70, 
	REG_L1_GAIN=0x71, 
	REG_L3_GAIN=0x72, 
	REG_L1_INT=0x73, 
	REG_L3_INT=0x74,

    REG_FIFO_DATA=0x60
  };

  // ===== 디버그 =====
  void debug_after_init(Stream& s);
};
