// =============================
// File: README.md
// =============================
# Notes
- Feature flags live in `config/Features.h`.
- Pins are centralized in `config/Pins.h`.
- MQ-2 math is wrapped in `drivers/MQ2.{h,cpp}` with divider + Rs/R0 logic identical to your original.
- Replace any CO ppm mapping with your actual calibration if needed.
- If you enable `USE_ICS43434`, avoid colliding those pins with UART2/ZE07.
- JSON is printed once per second (Timer100ms ISR → main loop flags).


- GPIO1 스위치가 눌려진 상태로 RESET 또는 5초간 누르고 있으면 SOFTAP 모드로 진입
  - 무선 AP를 로 연결하고, 웹브라우저에서 192.168.4.1을 입력  

- 초기 가열이 필요한 센서(MQ-2, SMOKE2 등) 사용 시, 센서가 안정화된 이후에 MQTT 데이터 전송을 시작합니다.

- 설정 페이지(SoftAP 모드)에서 MQ-2 센서의 현재 저항(Rs), 저장된 기준 저항(R0), 그리고 그 비율(Rs/R0)을 실시간으로 확인할 수 있습니다.

- SMOKE 2 연기감지센서 동작 Calibration