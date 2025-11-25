#pragma once
#include <Arduino.h>

namespace tmr100 {

// === 구버전 호환용 전역 플래그 ===
// 100ms * 10 = 1초마다 true가 됨. 사용 후 반드시 false로 클리어하세요.
// 예) if (tmr100::tick1000ms) { tmr100::tick1000ms = false; ... }
extern volatile bool tick1000ms;

// 타이머 시작/정지
void init();     // 100ms 주기로 내부 카운팅 (1초마다 tick1000ms = true)
void deinit();

// 편의 함수(선택 사용): 100ms/1s 이벤트 소비
bool consumeTick100ms();  // 100ms 틱이 1회 이상 누적되어 있으면 true 반환 후 소비
bool consume1s();         // 1초 이벤트가 발생했으면 true 반환 후 소비

// 디버그/통계용
uint32_t tickCount100ms();  // 누적 100ms 틱 카운트(오버플로우 주의)

} // namespace tmr100
