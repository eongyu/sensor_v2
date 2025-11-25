#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cSps30.h>

// --- 출력 포맷 매크로 정규화: FLOAT 이름이 없으면 값으로 보정 ---
#if defined(SPS30_OUTPUT_FORMAT_FLOAT)
  #define SPS30X_FMT_FLOAT SPS30_OUTPUT_FORMAT_FLOAT
#elif defined(SPS30_OUTPUT_FORMAT_FLOATING)
  #define SPS30X_FMT_FLOAT SPS30_OUTPUT_FORMAT_FLOATING
#else
  // 라이브러리 열거형이 없더라도 값은 동일(0x0301 == FLOAT)
  #define SPS30X_FMT_FLOAT ((SPS30OutputFormat)0x0301)
#endif

class SPS30X {
	public:
		// 기본 I2C 주소 0x69
		bool begin(TwoWire& w = Wire, uint8_t i2c_addr = 0x69) {
			_wire = &w;
			_sps.begin(*_wire, i2c_addr);

			_sps.stopMeasurement();
			delay(10);

			// 항상 FLOAT 포맷으로 시작 (라이브러리 v1.0.1 기준 지원됨)
			int16_t err = _sps.startMeasurement(SPS30X_FMT_FLOAT);
			if (err) return false;

			_started = true;
			_lastReadMs = 0;
			return true;
		}

		// μg/m³를 uint16로 반환(반올림/클램프). 성공 시 true
		bool read(uint16_t& pm1_0, uint16_t& pm2_5, uint16_t& pm4_0, uint16_t& pm10) {
			if (!_started) return false;
			//const uint32_t now = millis();
			//if (now - _lastReadMs < 900) return false; // 1초 간격 권장

			float mc_1p0, mc_2p5, mc_4p0, mc_10p0;
			float nc_0p5, nc_1p0, nc_2p5, nc_4p0, nc_10p0, typical_size;

			int16_t err = _sps.readMeasurementValuesFloat(
			mc_1p0, mc_2p5, mc_4p0, mc_10p0,
			nc_0p5, nc_1p0, nc_2p5, nc_4p0, nc_10p0,
			typical_size
			);
			if (err) {
				// 드물게 센서가 멈출 때 재시작
				_sps.stopMeasurement(); delay(5);
				_sps.startMeasurement(SPS30X_FMT_FLOAT);
				return false;
			}

			auto clamp = [](float x)->float {
				if (!isfinite(x) || x < 0) return 0.0f;
				if (x > 2000.0f) return 2000.0f; // 보호용 상한
				return x;
			};

			pm1_0 = (uint16_t)lroundf(clamp(mc_1p0));
			pm2_5 = (uint16_t)lroundf(clamp(mc_2p5));
			pm4_0 = (uint16_t)lroundf(clamp(mc_4p0));
			pm10  = (uint16_t)lroundf(clamp(mc_10p0));

			//_lastReadMs = now;
			return true;
		}

	private:
		SensirionI2cSps30 _sps;
		TwoWire* _wire = nullptr;
		bool _started = false;
		uint32_t _lastReadMs = 0;
};
