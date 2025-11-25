#include "Timer100ms.h"

// =================== 공통 상태 ===================
// (익명 네임스페이스: 이 파일 안에서만 접근 가능)
namespace {
// 구버전 호환용 플래그 (ait_wifi.ino에서 더 이상 직접 사용하지 않음)
volatile bool tick1000ms_compat = false;
// 100ms 단위 틱 누적
volatile uint32_t g_ticks_100ms = 0;
// 100ms 이벤트 발생 플래그
volatile bool g_tick100ms_flag = false;
// 1초(=100ms*10) 누적용
volatile uint8_t g_100ms_accum = 0;
// 1초 이벤트 플래그 (consume1s()용)
volatile bool g_1s_flag = false;

// 공통: ISR/타이머 콜백에서 호출되어 100ms 틱 처리
inline void IRAM_ATTR set_100ms_tick() {
  g_ticks_100ms++;
  g_tick100ms_flag = true;

  // 1초 누적
  g_100ms_accum++;
  if (g_100ms_accum >= 10) {     // 100ms * 10 = 1s
    g_100ms_accum = 0;
    g_1s_flag = true;
    // 구버전 호환 플래그 세트
    tick1000ms_compat = true;
  }
}

} // anonymous namespace

// ====== 구버전 호환용 플래그 정의 (namespace tmr100 안에서 정의) ======
namespace tmr100 {
// 이 네임스페이스의 함수들은 외부에서 호출 가능합니다.

// ========= 버전 분기 =========
// ESP32 Arduino Core 3.x 이상: timerBegin(frequency), timerAttachInterrupt(timer,isr), timerStart(timer)
// ESP32 Arduino Core 2.x 이하: timerBegin(timer, prescaler, countUp), timerAlarmWrite/Enable 등
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,0))

// =======================
// ESP32 Arduino Core 3.x
// =======================
namespace {
  hw_timer_t* g_timer = nullptr;

  void IRAM_ATTR onIsr3x() {
    set_100ms_tick();
  }
}

void init() {
  // 100ms = 10 Hz
  const uint32_t kHz = 10;
  g_timer = timerBegin(kHz);            // 3.x: 주파수(Hz)만 지정
  timerAttachInterrupt(g_timer, &onIsr3x);
  timerStart(g_timer);
}

void deinit() {
  if (g_timer) {
    timerStop(g_timer);
    timerDetachInterrupt(g_timer);
    // (3.x에서도 timerEnd가 있을 수 있으나, 안전하게 stop/detach까지만)
    g_timer = nullptr;
  }
}

bool consumeTick100ms() {
  if (g_tick100ms_flag) {
    g_tick100ms_flag = false;
    return true;
  }
  return false;
}

bool consume1s() {
  if (g_1s_flag) {
    g_1s_flag = false;
    // 호환용 플래그도 함께 클리어
    tick1000ms_compat = false;
    return true;
  }
  return false;
}

uint32_t tickCount100ms() { return g_ticks_100ms; }

#else

// =======================
// ESP32 Arduino Core 2.x
// =======================
namespace {
  hw_timer_t* g_timer = nullptr;
  // 2.x에서는 prescaler와 alarm을 직접 설정
  // APB 80MHz / 80 = 1us tick
  constexpr uint32_t kPrescaler = 80;     // 1us
  constexpr uint64_t kAlarmUs   = 100000; // 100ms

  void IRAM_ATTR onIsr2x() {
    set_100ms_tick();
  }
}

void init() {
  g_timer = timerBegin(0, kPrescaler, true);    // timer 0, 1us tick, countUp
  timerAttachInterrupt(g_timer, &onIsr2x, true);
  timerAlarmWrite(g_timer, kAlarmUs, true);     // 100ms 주기 반복
  timerAlarmEnable(g_timer);
}

void deinit() {
  if (g_timer) {
    timerAlarmDisable(g_timer);
    timerDetachInterrupt(g_timer);
    timerEnd(g_timer);
    g_timer = nullptr;
  }
}

bool consumeTick100ms() {
  if (g_tick100ms_flag) {
    g_tick100ms_flag = false;
    return true;
  }
  return false;
}

bool consume1s() {
  if (g_1s_flag) {
    g_1s_flag = false;
    // 호환용 플래그도 함께 클리어
    tick1000ms_compat = false;
    return true;
  }
  return false;
}

uint32_t tickCount100ms() { return g_ticks_100ms; }

#endif

} // namespace tmr100
