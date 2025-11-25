// =============================
// File: ait_wifi.ino
//	Last Update : 2025/11/20
// =============================

// --- Standard & System ---
#include <Arduino.h>
#include <cmath> // pow() 함수를 사용하기 위해 추가
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <StreamString.h>
#include <Preferences.h>
#include <WebServer.h>
#include <Update.h>
#include <DNSServer.h>
#include <esp_task_wdt.h> // Watchdog Timer 리셋을 위해 추가



// --- Project Config & Core ---
#include "src/config/BuildOpts.h"
#include "src/config/Pins.h"
#include "src/config/Features.h"
#include "src/core/LEDs.h"
#include "src/core/Timer100ms.h"
#include "src/core/JsonOut.h"

// --- Project Drivers ---
#include "src/drivers/ADS1115_Helper.h"
#include "src/drivers/MQ2.h"
#include "src/drivers/ZE07.h"
#include "src/drivers/SEN0177.h"
#include "src/drivers/BME68X.h"
#include "src/drivers/SGP30X.h"
#include "src/drivers/SMOKE2.h"
#include "src/drivers/SPS30X.h" // 이전 수정에서 추가된 SPS30X도 경로 변경
#include "src/drivers/ICS43434X.h" // ICS43434도 경로 변경

// FreeRTOS 헤더
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// =================================================================
// Default Configuration Values
// =================================================================
#define DEFAULT_WIFI_SSID "IAE_ROBOT_LAB_2G"
#define DEFAULT_WIFI_PASS "robotics7607"
#define DEFAULT_MQTT_HOST "192.168.0.186"
#define DEFAULT_MQTT_PORT 1883
#define DEFAULT_MQ2_R0    9000.0f


// =================================================================
// Configuration Management
// =================================================================

// 설정 구조의 버전을 관리하기 위한 ID (2-byte)
#define CONFIG_VERSION 0x0001

// 설정 값을 담을 구조체
struct AppConfig {
	char wifi_ssid[33];
	char wifi_pass[65];
	char mqtt_host[65];
	uint16_t mqtt_port;
	char mqtt_user[33];
	char mqtt_pass[65];
	float mq2_r0; // MQ-2 센서의 기준 저항(R0) 값
	float smoke2_alpha; // SMOKE2 센서의 기준 값(alpha)
};

AppConfig g_config; // 전역 설정 변수
Preferences g_prefs; // NVS 저장을 위한 Preferences 객체
static WebServer g_server(80); // 설정 웹 서버
static DNSServer g_dnsServer; // Captive Portal을 위한 DNS 서버

void saveConfiguration();
void loadConfiguration();
void startConfigPortal();

// =================================================================
// WiFi & MQTT
// =================================================================

// ---- 기본 토픽 설정 ----
#ifndef MQTT_TOPIC
#define MQTT_TOPIC "sensorhub/telemetry"
#endif
#ifndef MQTT_STATUS_TOPIC
#define MQTT_STATUS_TOPIC "sensorhub/status"
#endif

// 강제 설정 모드 진입을 위한 버튼 핀
#define PIN_FORCE_CONFIG_PORTAL 1

#define USE_DEBUG	0

static WiFiClient g_net;
static PubSubClient g_mqtt(g_net);
static uint32_t g_lastConnMs = 0;

// FreeRTOS 핸들
static TaskHandle_t g_sensorTaskHandle = NULL;
static TaskHandle_t g_mqttTaskHandle = NULL;
// 센서 Task -> MQTT Task로 JSON 문자열을 전달하기 위한 Queue
static uint32_t g_buttonPressStartTime = 0;
const uint32_t CONFIG_PORTAL_HOLD_TIME_MS = 5000; // 5 seconds

static QueueHandle_t g_mqttQueue = NULL;
// Queue에 저장할 메시지의 최대 길이
#define MAX_JSON_MSG_SIZE 2048


static void i2cInit(){ 
	Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ); 
}


// Globals
#if USE_ADS1115 || USE_CO_ADC
	ADS1115_Helper ads;
#endif

#if USE_MQ2
	MQ2 mq2; MQ2Config mq2cfg;
#endif

#if USE_ZE07
	ZE07 ze07;
	HardwareSerial CO_SER(2);
#endif

#if USE_SEN0177
	SEN0177 sen0177;
	HardwareSerial PM_SER(1);
#endif

#if USE_BME688
	BME68X bme688;
#endif

#if USE_SGP30
	SGP30X sgp30;
#endif

#if USE_SPS30
	SPS30X sps30;
#endif

#if USE_SMOKE2
	SMOKE2 smoke2;
#endif

#if USE_ICS43434
	ICS43434X mic;
#endif


static void wifi_connect_blocking(){
	if(WiFi.status() == WL_CONNECTED) return;
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true);
	WiFi.persistent(false);
	WiFi.begin(g_config.wifi_ssid, g_config.wifi_pass);
	uint32_t t0 = millis();
	while(WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000){
		Serial.print(".");
		delay(200);
	}
}

static void mqtt_connect_nonblock(){
	if(g_mqtt.connected()) return;
	uint32_t now = millis();
	if(now - g_lastConnMs < 2000) return; // simple backoff
	g_lastConnMs = now;
	g_mqtt.setServer(g_config.mqtt_host, g_config.mqtt_port);

	// LWT(유언) 메시지 설정: 연결이 비정상적으로 끊기면 브로커가 "offline" 메시지를 발행
	const bool willRetain = true;
	const uint8_t willQos = 1;
	const char* willTopic = MQTT_STATUS_TOPIC;
	const char* willPayload = "offline";

	if(strlen(g_config.mqtt_user) > 0){
		g_mqtt.connect("ait-node", g_config.mqtt_user, g_config.mqtt_pass, willTopic, willQos, willRetain, willPayload);
	}else{
		g_mqtt.connect("ait-node", willTopic, willQos, willRetain, willPayload);
	}
}

static void net_loop(){
	if(WiFi.status() != WL_CONNECTED){
		wifi_connect_blocking();
	}
	if(!g_mqtt.connected()){
		mqtt_connect_nonblock();
	}
	g_mqtt.loop();
}

// =================================================================
// FreeRTOS Tasks
// =================================================================

/**
 * @brief 1초마다 센서 값을 읽어 JSON으로 만들고 Queue에 전송하는 Task
 * @param pvParameters Task 파라미터 (사용 안 함)
 */
void sensorTask(void *pvParameters) {
	bool g_systemReady = false; // 모든 센서가 안정화되었는지 확인하는 플래그

	Serial.println("Sensor Task: started");
	bool led3 = false;

	// vTaskDelayUntil을 위한 변수 초기화
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1000ms 주기
	xLastWakeTime = xTaskGetTickCount(); // 현재 시간을 기준으로 시작

	for (;;) {
		// 정확한 1초 주기를 위해 다음 실행 시간까지 대기
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// 시스템이 아직 준비되지 않았다면, 센서들의 준비 상태를 확인
		if (!g_systemReady) {
			bool mq2_ready = false;
			bool bme688_ready = false;
			bool smoke2_ready = false;

			#if USE_MQ2
				mq2_ready = (mq2.phase() == MQ2::RUN);
			#else
				mq2_ready = true;
			#endif

			#if USE_BME688
				// bme.isReading() 과 같은 명시적인 준비 함수가 없으므로,
				// 첫 번째 유효한 가스 저항 값(0 이상)을 받으면 준비된 것으로 간주합니다.
				bme688_ready = (bme688.last_gas_resistance() > 0);
			#else
				bme688_ready = true;
			#endif

			#if USE_SMOKE2
				smoke2_ready = smoke2.isBaselineReady();
			#else
				smoke2_ready = true;
			#endif

			if (mq2_ready && bme688_ready && smoke2_ready) {
				g_systemReady = true;
				Serial.println("[SYSTEM] All sensors are ready. Starting MQTT publish.");
			} else {
				// 디버깅: 어떤 센서가 아직 준비되지 않았는지 확인
				Serial.printf("[SYSTEM] Waiting for sensors... MQ2: %d, BME688: %d, SMOKE2: %d\n",
					(int)mq2_ready, (int)bme688_ready, (int)smoke2_ready
				);
			}
		}

		//led3 = !led3;
		leds::blink3(1);

		StreamString json_buf;
		// JSON 부분은 반드시 { ... }로 사용해야 함
		{
			JsonOut js(json_buf);

			#if USE_SPS30
			static uint16_t pm1, pm25, pm4, pm10;
			if(sps30.read(pm1,pm25,pm4,pm10)){ js.add("pm1_0",pm1); js.add("pm2_5",pm25); js.add("pm4_0",pm4); js.add("pm10",pm10); }
			#endif

			#if USE_BME688
			float t,h,g; 
			if(bme688.read(t,h,g)){ js.add("temp",t); js.add("hum",h); float g_kohm = roundf((g * 0.001f) * 1000.0f) / 1000.0f; js.add("gas_kohm",g_kohm,3); }
			#endif

			#if USE_SMOKE2
			SMOKE2::Reading sr;
			if (smoke2.read(sr)) { js.addU("smk_blue", (uint32_t)sr.blue); js.addU("smk_ir", (uint32_t)sr.ir); js.add("smk_ratio", sr.ratio, 3); js.add("smk_alpha", sr.alpha, 3); js.add("smk_score", sr.score, 0); js.addU("smk_alarm", sr.alarm ? 1u : 0u); }
			#endif

			#if USE_SGP30
			uint16_t eco2,tvoc; 
			if(sgp30.read(eco2,tvoc)){ js.addU("eCO2_ppm",eco2); js.addU("TVOC_ppb",tvoc); }
			#endif

			#if USE_CO_ADC   // GSET11-P110 테이블 기준

			// 노이즈에 더 강한 'Trimmed Mean' 방식 적용
			// 10회 측정 후 최소/최대 2개씩 제외하고 6개 값으로 평균 계산
			const int SAMPLE_COUNT = 10;
			float samples[SAMPLE_COUNT];

			for (int i = 0; i < SAMPLE_COUNT; i++) {
				samples[i] = ads.read_V(0);
				delay(10); // 10ms 대기
			}

			// 측정된 샘플을 오름차순으로 정렬 (간단한 삽입 정렬 사용)
			for (int i = 1; i < SAMPLE_COUNT; i++) {
				float key = samples[i];
				int j = i - 1;
				while (j >= 0 && samples[j] > key) {
					samples[j + 1] = samples[j];
					j = j - 1;
				}
				samples[j + 1] = key;
			}

			// 최소값 2개, 최대값 2개를 제외한 나머지 6개의 합계를 계산
			float co_V_sum = 0.0f;
			for (int i = 2; i < SAMPLE_COUNT - 2; i++) {
				co_V_sum += samples[i];
			}
			float co_V = co_V_sum / (SAMPLE_COUNT - 4); // 6개 값의 평균
			float CO_ppm;

			// 1) 너무 낮은 전압은 0ppm으로 처리 (노이즈/오류 방지용)
			if (co_V <= 1.0f) {
				CO_ppm = 0.0f;

			// 2) 구간1 : 1.0 ~ 1.82 V (대략 0~100 ppm)
			} else if (co_V < 1.82f) {
				CO_ppm = 151.864662f * co_V * co_V
					- 303.866154f * co_V
					+ 153.910337f;

			// 3) 구간2 : 1.82 ~ 2.18 V (대략 110~220 ppm)
			} else if (co_V < 2.18f) {
				CO_ppm = 127.987999f * co_V * co_V
					- 177.634899f * co_V
					+ 2.772567f;

			// 4) 구간3 : 2.18 ~ 2.61 V (대략 230~400 ppm)
			} else if (co_V < 2.61f) {
				CO_ppm = 134.327926f * co_V * co_V
					- 177.884252f * co_V
					- 26.303081f;

			// 5) 구간4 : 2.61 ~ 3.55 V (대략 450~1000 ppm)
			} else {
				CO_ppm = 79.481009f * co_V * co_V
					+ 123.214449f * co_V
					- 439.821071f;
			}

			// 6) 안전하게 범위 클램프
			if (CO_ppm < 0.0f)    CO_ppm = 0.0f;
			if (CO_ppm > 1000.0f) CO_ppm = 1000.0f;

			js.add("CO_V", co_V);	
			js.add("CO_ppm", CO_ppm);

		#endif

			#if USE_ADS1115 && USE_MQ2
			float mq_mV = ads.read_mV(1); mq2.update_from_adc_mV(mq_mV); js.add("mq2_mV", mq_mV); js.add("mq2_Rs", mq2.rs(),0); js.add("mq2_ratio", mq2.ratio()); js.add("mq2_ema", mq2.ratio_ema()); if(mq2.alarm()) leds::set3(true); else leds::set3(false);
			#endif

			#if USE_ADS1115
			js.add("Smoke_mV", ads.read_mV(2));
			#endif

			#if USE_ICS43434
			js.add("mic_rms", mic.read_rms(),0);
			#endif

			#if USE_SEN0177
			PM25Data d; if(sen0177.read(d)){ js.addU("pm1_0", d.pm1_0); js.addU("pm2_5", d.pm2_5); js.addU("pm10", d.pm10); }
			#endif

			#if USE_ZE07
			if(ze07.read_frame(300)){ float ppm=0; uint16_t full=0; uint8_t dec=0; if(ze07.parse_ppm(ppm, full, dec)){ js.add("ZE07_CO_ppm", ppm, 1); } }
			#endif
		}

		// 생성된 JSON 문자열을 Queue로 전송
		if (g_systemReady) {
			if (json_buf.length() > 0) {
				char msg_payload[MAX_JSON_MSG_SIZE]; // 스택에 버퍼 할당
				// strncpy로 안전하게 복사 (메모리 누수 위험 없음)
				strncpy(msg_payload, json_buf.c_str(), MAX_JSON_MSG_SIZE - 1);
				msg_payload[MAX_JSON_MSG_SIZE - 1] = '\0'; // 항상 NULL로 종료 보장

				// 10ms 타임아웃으로 Queue에 전송 시도
				// xQueueSend는 메시지를 '복사'하므로, 함수가 반환된 후 msg_payload는 안전하게 파괴됨
				if (xQueueSend(g_mqttQueue, msg_payload, pdMS_TO_TICKS(10)) != pdPASS) {
					Serial.println("Sensor Task: Failed to send to queue, queue full?");
				}
			}
		}
	}
}

/**
 * @brief WiFi/MQTT 연결을 관리하고, Queue에 데이터가 오면 publish하는 Task
 * @param pvParameters Task 파라미터 (사용 안 함)
 */
void mqttTask(void *pvParameters) {
	Serial.println("MQTT Task: started");
	char received_payload[MAX_JSON_MSG_SIZE];
	bool birth_message_published = false; // Birth 메시지가 발행되었는지 추적하는 플래그

	for (;;) {
		// WiFi 및 MQTT 연결 관리
		// net_loop()는 내부적으로 연결이 끊겼을 때만 mqtt_connect_nonblock()을 호출합니다.
		net_loop();
		
		// Queue에서 메시지를 기다림 (최대 100ms 대기)
		if (xQueueReceive(g_mqttQueue, received_payload, pdMS_TO_TICKS(100)) == pdPASS) {
			if (g_mqtt.connected()) {
				g_mqtt.publish(MQTT_TOPIC, (const uint8_t*)received_payload, strlen(received_payload), false);
				#if USE_DEBUG
				Serial.print("[MQTT Task] Published: "); Serial.println(received_payload);
				#endif
			}
		}

		// MQTT가 연결되었고, 아직 Birth 메시지를 발행하지 않았다면 발행합니다.
		if (g_mqtt.connected() && !birth_message_published) {
			Serial.println("MQTT Task: Publishing birth message.");
			g_mqtt.publish(MQTT_STATUS_TOPIC, "online", true);
			birth_message_published = true; // 발행되었음을 표시하여 다시 발행하지 않도록 함
		}
	}
}

// =================================================================
// Configuration Portal Functions
// =================================================================

void loadConfiguration() {
	g_prefs.begin("sensor-hub", false);

	// 1. 저장된 설정 버전 확인
	uint16_t saved_version = g_prefs.getUShort("cfg_version", 0);

	// 2. 버전이 다르거나 처음 부팅하는 경우, 모든 설정을 기본값으로 초기화
	if (saved_version != CONFIG_VERSION) {
		Serial.printf("[CONFIG] Version mismatch or new device. Initializing with defaults. (Saved: 0x%04X, Code: 0x%04X)\n", saved_version, CONFIG_VERSION);

		// 정의된 매크로를 사용하여 기본값 설정
		strcpy(g_config.wifi_ssid, DEFAULT_WIFI_SSID);
		strcpy(g_config.wifi_pass, DEFAULT_WIFI_PASS);
		strcpy(g_config.mqtt_host, DEFAULT_MQTT_HOST);
		g_config.mqtt_port = DEFAULT_MQTT_PORT;
		strcpy(g_config.mqtt_user, "");
		strcpy(g_config.mqtt_pass, "");
		g_config.mq2_r0 = DEFAULT_MQ2_R0;

		// 이 기본값들을 NVS에 즉시 저장
		saveConfiguration();
	} else {
		// 3. 버전이 같으면, 저장된 사용자 설정 값을 로드
		g_prefs.getString("wifi_ssid", g_config.wifi_ssid, sizeof(g_config.wifi_ssid));
		g_prefs.getString("wifi_pass", g_config.wifi_pass, sizeof(g_config.wifi_pass));
		g_prefs.getString("mqtt_host", g_config.mqtt_host, sizeof(g_config.mqtt_host));
		g_config.mqtt_port = g_prefs.getUShort("mqtt_port", 1883);
		g_prefs.getString("mqtt_user", g_config.mqtt_user, sizeof(g_config.mqtt_user));
		g_prefs.getString("mqtt_pass", g_config.mqtt_pass, sizeof(g_config.mqtt_pass));
		g_config.mq2_r0 = g_prefs.getFloat("mq2_r0", DEFAULT_MQ2_R0);
		g_config.smoke2_alpha = g_prefs.getFloat("smoke2_alpha", 0.0f); // 0.0f는 아직 교정되지 않았음을 의미
	}
	g_prefs.end();
}

void saveConfiguration() {
	g_prefs.begin("sensor-hub", false);
	g_prefs.putUShort("cfg_version", CONFIG_VERSION); // 현재 설정 버전을 함께 저장
	g_prefs.putString("wifi_ssid", g_config.wifi_ssid);
	g_prefs.putString("wifi_pass", g_config.wifi_pass);
	g_prefs.putString("mqtt_host", g_config.mqtt_host);
	g_prefs.putUShort("mqtt_port", g_config.mqtt_port);
	g_prefs.putString("mqtt_user", g_config.mqtt_user);
	g_prefs.putString("mqtt_pass", g_config.mqtt_pass);
	g_prefs.putFloat("mq2_r0", g_config.mq2_r0);
	g_prefs.putFloat("smoke2_alpha", g_config.smoke2_alpha);
	g_prefs.end();
}

void startConfigPortal() {
	const char* ap_ssid = "SensorHub-Config";
	Serial.println("\nStarting Configuration Portal.");
	Serial.printf("Connect to WiFi AP: %s\n", ap_ssid);

	WiFi.softAP(ap_ssid);
	IPAddress apIP = WiFi.softAPIP();
	Serial.printf("AP IP address: %s\n", apIP.toString().c_str());

	// Captive Portal을 위해 DNS 서버 시작
	g_dnsServer.start(53, "*", apIP);
	leds::blink1(1);
	leds::blink2(1);
	leds::blink3(1);
	leds::blink4(1);

	// 루트 페이지 및 Captive Portal을 위한 "Catch-all" 핸들러
	auto handleRoot = []() {
		String html = R"rawliteral(
			<!DOCTYPE html><html><head><title>SensorHub Config</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background-color: #f4f6f8; color: #333; margin: 0; padding: 20px; display: flex; justify-content: center; align-items: center; min-height: 100vh; }
                .container { background-color: #fff; padding: 30px; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); max-width: 400px; width: 100%; }
                h1, h2 { color: #1a2533; text-align: center; border-bottom: 1px solid #eee; padding-bottom: 10px; margin-bottom: 20px; }
                h1 { font-size: 1.8em; }
                h2 { font-size: 1.2em; margin-top: 30px; }
                p { text-align: center; margin-top: -10px; margin-bottom: 30px; color: #888; }
                label { display: block; margin-top: 15px; font-weight: bold; font-size: 0.9em; }
                input { width: 100%; padding: 12px; margin-top: 5px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
                .btn-container { display: flex; gap: 10px; margin-top: 30px; }
                button { background-color: #007aff; color: white; padding: 14px 20px; border: none; border-radius: 4px; cursor: pointer; width: 100%; font-size: 1em; font-weight: bold; }
                button:hover { background-color: #0056b3; }
                button.secondary { background-color: #6c757d; }
                button.secondary:hover { background-color: #5a6268; }
                a { color: #007aff; text-decoration: none; display: block; text-align: center; margin-top: 20px; }
            </style>
            </head><body>
            <div class="container">
			    <form action="/save" method="POST">
				<h1>SensorHub Configuration</h1>
				
                <h2>WiFi Settings</h2>
				<label for="ssid">SSID</label>
				<input type="text" id="ssid" name="ssid" value=")rawliteral";
		html += g_config.wifi_ssid;
		html += R"rawliteral(">
				<label for="pass">Password</label>
				<input type="password" id="pass" name="pass">

				<h2>MQTT Settings</h2>
				<label for="host">Broker Host</label>
				<input type="text" id="host" name="host" value=")rawliteral";
		html += g_config.mqtt_host;
		html += R"rawliteral(">
				<label for="port">Port</label>
				<input type="number" id="port" name="port" value=")rawliteral";
		html += String(g_config.mqtt_port);
		html += R"rawliteral(">
				<label for="user">User (optional)</label>
				<input type="text" id="user" name="user" value=")rawliteral";
		html += g_config.mqtt_user;
		html += R"rawliteral(">
				<label for="m_pass">Password (optional)</label>
				<input type="password" id="m_pass" name="m_pass">

				<h2>Sensor Calibration</h2>
				<p>Place the device in clean air before calibrating.</p>
				<button type="button" id="calibBtn" class="secondary">Calibrate MQ-2 R0</button>
				<p id="calib_status"></p>
				<br>
				<button type="button" id="calibSmokeBtn" class="secondary">Calibrate SMOKE 2 Alpha</button>
				<p id="calib_status"></p>

                <div class="btn-container">
				    <button type="submit">Save & Reboot</button>
                </div>

				<h2>Live Sensor Status</h2>
				<p id="status_area">Loading...</p>
				    <button type="button" id="readBtn" class="secondary">Read Saved</button>

                <a href="/update">Go to Firmware Update</a>
			</form>
            </div></body></html>
            <script>
                document.getElementById('calibBtn').addEventListener('click', function() {
                    const statusEl = document.getElementById('calib_status');
                    statusEl.innerText = 'Calibrating... Please wait about 30 seconds.';
                    this.disabled = true;

                    fetch('/calibrate_mq2')
                        .then(response => response.text())
                        .then(data => {
                            statusEl.innerText = data;
                            this.disabled = false;
                        })
                        .catch(error => {
                            console.error('Error:', error);
                            statusEl.innerText = 'Calibration failed. Please try again.';
                            this.disabled = false;
                        });
                });

                document.getElementById('calibSmokeBtn').addEventListener('click', function() {
                    const statusEl = document.getElementById('calib_status');
                    statusEl.innerText = 'Calibrating SMOKE 2... Please wait about 20 seconds.';
                    this.disabled = true;

                    fetch('/calibrate_smoke2')
                        .then(response => response.text())
                        .then(data => {
                            statusEl.innerText = data;
                            this.disabled = false;
                        })
                        .catch(error => {
                            statusEl.innerText = 'SMOKE 2 calibration failed.';
                            this.disabled = false;
                        });
                });

                document.getElementById('readBtn').addEventListener('click', function() {
                    fetch('/readconfig')
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('ssid').value = data.ssid;
                            document.getElementById('host').value = data.host;
                            document.getElementById('port').value = data.port;
                            document.getElementById('user').value = data.user;
                            // 보안을 위해 저장된 비밀번호는 다시 불러오지 않고, 입력 필드를 비웁니다.
                            document.getElementById('pass').value = '';
                            document.getElementById('m_pass').value = '';
                            alert('Saved values have been loaded. Passwords are not shown for security.');
                        })
                        .catch(error => console.error('Error:', error));
                });

                function fetchStatus() {
                    fetch('/status')
                        .then(response => response.json())
                        .then(data => {
                            const statusEl = document.getElementById('status_area');
                            let statusText = `MQ-2 R0 (Saved): ${data.mq2_r0.toFixed(0)} &Omega;<br>`;
                            statusText += `MQ-2 Rs (Live): ${data.mq2_rs.toFixed(0)} &Omega;<br>`;
                            statusText += `Rs/R0 Ratio: ${data.mq2_ratio.toFixed(3)}`;
                            statusText += `<hr>SMOKE2 Alpha (Saved): ${data.smoke2_alpha.toFixed(4)}<br>`;
                            statusText += `SMOKE2 Ratio (Live): ${data.smoke2_ratio.toFixed(4)}<br>`;
                            statusText += `Score: ${data.smoke2_score.toFixed(0)}`;
                            statusEl.innerHTML = statusText;
                        })
                        .catch(error => {
                            document.getElementById('status_area').innerText = 'Failed to load status.';
                        });
                }
                setInterval(fetchStatus, 5000); // 5초마다 상태 업데이트
                fetchStatus(); // 페이지 로드 시 즉시 실행
            </script>
		)rawliteral";
		g_server.send(200, "text/html", html);
	};

	// 웹 서버 저장 페이지 핸들러
	auto handleSave = []() {
		strncpy(g_config.wifi_ssid, g_server.arg("ssid").c_str(), sizeof(g_config.wifi_ssid));
		strncpy(g_config.wifi_pass, g_server.arg("pass").c_str(), sizeof(g_config.wifi_pass));
		strncpy(g_config.mqtt_host, g_server.arg("host").c_str(), sizeof(g_config.mqtt_host));
		g_config.mqtt_port = g_server.arg("port").toInt();
		strncpy(g_config.mqtt_user, g_server.arg("user").c_str(), sizeof(g_config.mqtt_user));
		strncpy(g_config.mqtt_pass, g_server.arg("m_pass").c_str(), sizeof(g_config.mqtt_pass));

		saveConfiguration();

		String html = "<html><body><h1>Settings Saved.</h1><h2>Rebooting...</h2></body></html>";
		g_server.send(200, "text/html", html);

		delay(1000);
		ESP.restart();
	};

	// MQ-2 R0 교정 핸들러
	auto handleCalibrate = []() {
		g_server.send(200, "text/plain", "Calibrating... This will take about 30 seconds.");

		const int calib_samples = 30;
		float r0_sum = 0.0f;

		// 워밍업을 위해 잠시 대기
		delay(1000);

		for (int i = 0; i < calib_samples; i++) {
			mq2.update_from_adc_mV(ads.read_mV(1));
			r0_sum += mq2.rs();
			// Watchdog Timer 리셋
			esp_task_wdt_reset();
			// 1초 대기하는 동안 Watchdog Timer가 리셋되지 않아 재부팅되는 것을 방지하기 위해
			// 주기적으로 Watchdog을 리셋해줍니다.
			delay(1000); // 1초 간격으로 측정
		}

		float new_r0 = r0_sum / calib_samples;
		g_config.mq2_r0 = new_r0;
		saveConfiguration();

		String response = "Calibration complete! New R0 value: " + String(new_r0, 2) + " Ohms. Saved to memory.";
		// 클라이언트가 이미 200 OK 응답을 받았으므로, 이 응답은 보낼 수 없음.
		// 대신, 다음 요청 시 클라이언트가 이 상태를 확인할 수 있도록 함.
		Serial.println(response);
	};

	// SMOKE2 Alpha 교정 핸들러
	auto handleCalibrateSmoke2 = []() {
		g_server.send(200, "text/plain", "Calibrating SMOKE 2... This will take about 20 seconds.");

		// 드라이버의 예열/교정 로직을 강제로 다시 실행
		smoke2.begin(Wire, 0x64, 0.0f); // 0.0f를 전달하여 재교정 모드로 시작

		// 20초간 대기하며 read()를 호출하여 베이스라인을 생성
		for (int i = 0; i < 20; i++) {
			SMOKE2::Reading r;
			smoke2.read(r);
			Serial.print("Timer : ");
			Serial.println(i);
			// Watchdog Timer가 리셋되지 않아 재부팅되는 것을 방지
			esp_task_wdt_reset();
			delay(1000);
		}

		g_config.smoke2_alpha = smoke2.getAlpha();
		saveConfiguration();
		String response = "Calibration complete! New SMOKE2 alpha: " + String(g_config.smoke2_alpha, 4) + ". Saved to memory.";
		// 클라이언트가 이미 200 OK 응답을 받았으므로, 이 응답은 보낼 수 없음.
		// 대신, 다음 요청 시 클라이언트가 이 상태를 확인할 수 있도록 함.
		Serial.println(response);
	};

	// 펌웨어 업데이트 페이지 핸들러
	g_server.on("/update", HTTP_GET, []() {
		String html = R"rawliteral(
			<!DOCTYPE html><html><head><title>Firmware Update</title>
			<meta name="viewport" content="width=device-width, initial-scale=1">
			<style>
				body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background-color: #f4f6f8; color: #333; margin: 0; padding: 20px; display: flex; justify-content: center; align-items: center; min-height: 100vh; }
				.container { background-color: #fff; padding: 30px; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); max-width: 400px; width: 100%; text-align: center; }
				h1 { color: #1a2533; }
				input[type=file] { margin: 20px 0; }
				button { background-color: #ff9500; color: white; padding: 14px 20px; border: none; border-radius: 4px; cursor: pointer; width: 100%; font-size: 1em; font-weight: bold; }
				button:hover { background-color: #d67e00; }
                .progress { width: 100%; background-color: #ddd; border-radius: 4px; margin-top: 20px; }
                .bar { width: 0%; height: 20px; background-color: #007aff; border-radius: 4px; text-align: center; color: white; line-height: 20px; }
			</style></head><body>
			<div class="container">
				<h1>Firmware Update</h1>
				<form method="POST" action="/update" enctype="multipart/form-data">
					<input type="file" name="update">
					<button type="submit">Update</button>
				</form>
                <div class="progress"><div class="bar" id="progressBar">0%</div></div>
			</div>
            <script>
                document.querySelector('form').addEventListener('submit', function(e) {
                    e.preventDefault();
                    var formData = new FormData(this);
                    var xhr = new XMLHttpRequest();
                    xhr.open('POST', '/update', true);
                    xhr.upload.addEventListener('progress', function(e) {
                        if (e.lengthComputable) {
                            var percentComplete = (e.loaded / e.total) * 100;
                            var bar = document.getElementById('progressBar');
                            bar.style.width = percentComplete.toFixed(2) + '%';
                            bar.textContent = percentComplete.toFixed(2) + '%';
                        }
                    });
                    xhr.onload = function() {
                        if (xhr.status === 200) {
                            alert('Update Success! Rebooting...');
                            setTimeout(function(){ window.location.href = '/'; }, 1000);
                        } else {
                            alert('Update Failed: ' + xhr.responseText);
                        }
                    };
                    xhr.send(formData);
                });
            </script></body></html>)rawliteral";
		g_server.send(200, "text/html", html);
	});

	// 저장된 설정 값을 JSON으로 응답하는 핸들러
	g_server.on("/readconfig", HTTP_GET, []() {
		loadConfiguration(); // Flash에서 최신 설정 다시 로드
		String json = "{";
		json += "\"ssid\":\"" + String(g_config.wifi_ssid) + "\",";
		json += "\"host\":\"" + String(g_config.mqtt_host) + "\",";
		json += "\"port\":" + String(g_config.mqtt_port) + ",";
		json += "\"user\":\"" + String(g_config.mqtt_user) + "\"";
		// 보안상 비밀번호는 JSON 응답에 포함하지 않습니다.
		json += "}";
		g_server.send(200, "application/json", json);
	});

	// 실시간 센서 상태를 JSON으로 응답하는 핸들러
	auto handleStatus = []() {
		String json = "{";
		#if USE_ADS1115 && USE_MQ2
			// 설정 포털에서는 sensorTask가 돌지 않으므로, 직접 값을 읽고 계산합니다.
			float mq_mV = ads.read_mV(1);
			// update_from_adc_mV는 내부 상태를 바꾸므로, 여기서는 계산만 수행합니다.
			float current_rs = mq2.calc_Rs_from_AO_mV_(mq_mV);
			float current_r0 = g_config.mq2_r0 > 0 ? g_config.mq2_r0 : 1.0f;
			float ratio = current_rs / current_r0;

			json += "\"mq2_r0\":" + String(g_config.mq2_r0, 2) + ",";
			json += "\"mq2_rs\":" + String(current_rs, 2) + ",";
			json += "\"mq2_ratio\":" + String(ratio, 4);
		#if USE_SMOKE2
			SMOKE2::Reading sr;
			smoke2.read(sr);
			json += ",\"smoke2_alpha\":" + String(g_config.smoke2_alpha, 4) + ",";
			json += "\"smoke2_ratio\":" + String(sr.ratio, 4) + ",";
			json += "\"smoke2_score\":" + String(sr.score, 0);
		#else
			json += ",\"smoke2_alpha\":0, \"smoke2_ratio\":0, \"smoke2_score\":0";
		#endif

		#else
			json += "\"mq2_r0\":0, \"mq2_rs\":0, \"mq2_ratio\":0";
		#endif
		json += "}";
		g_server.send(200, "application/json", json);
	};
	// 핸들러 등록: 특정 경로를 먼저 등록하고, 그 외 모든 요청을 처리할 핸들러를 마지막에 등록합니다.
	g_server.on("/update", HTTP_POST, []() {
		g_server.sendHeader("Connection", "close");
		g_server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
		ESP.restart();
	}, []() {
		HTTPUpload& upload = g_server.upload();
		if (upload.status == UPLOAD_FILE_START) {
			Serial.printf("Update: %s\n", upload.filename.c_str());
			if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // 전체 크기를 모를 때
				Update.printError(Serial);
			}
		} else if (upload.status == UPLOAD_FILE_WRITE) {
			if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
				Update.printError(Serial);
			}
		} else if (upload.status == UPLOAD_FILE_END) {
			if (Update.end(true)) { // true to set the size to the current progress
				Serial.printf("Update Success: %u bytes\n", upload.totalSize);
			} else {
				Update.printError(Serial);
			}
		}
	});

	g_server.on("/save", HTTP_POST, handleSave);
	g_server.on("/calibrate_mq2", HTTP_GET, handleCalibrate);
	g_server.on("/calibrate_smoke2", HTTP_GET, handleCalibrateSmoke2);
	g_server.on("/status", HTTP_GET, handleStatus);
	g_server.on("/", HTTP_GET, handleRoot);
	g_server.on("/generate_204", HTTP_GET, handleRoot); // Android Captive Portal
	g_server.onNotFound(handleRoot); // 모든 나머지 요청 처리

	g_server.begin();

	// 설정 포털 루프
	while (true) {
		g_dnsServer.processNextRequest();
		g_server.handleClient();
		delay(10);
	}
}

void smoke2_one_shot_dump(){
  // 이 함수는 이제 SMOKE2 클래스의 public 디버깅 함수를 호출합니다.
  Serial.println("--- smoke2_one_shot_dump ---");
  smoke2.debug_fifo_probe(Serial);
}


void setup(){
	// 시리얼 초기화가 가장 먼저 와야 함
	Serial.begin(115200); 
	delay(50);
	Serial.println("\n\nBooting SensorHub...");

	// 센서 초기화 전에 반드시 설정을 먼저 로드해야 합니다.
	loadConfiguration();

	leds::init();

	// =================================================
	// 센서 초기화 (설정 포털에서도 사용 가능하도록 앞쪽으로 이동)
	// =================================================
	tmr100::init();
	i2cInit();
	analogReadResolution(ADC_RES_BITS);

	#if USE_ADS1115 || USE_CO_ADC
		if(!ads.begin()) Serial.println(F("[ADS1115] init failed"));
	#endif

	#if USE_BME688
		if(!bme688.begin(0x76)) Serial.println(F("[BME688] not found"));
	#endif

	#if USE_SGP30
		if(!sgp30.begin()) Serial.println(F("[SGP30] not found"));
	#endif

	#if USE_SMOKE2
		// 센서의 감도(TIA Gain)를 기본 200kΩ에서 1MΩ으로 5배 높입니다.
		// 이렇게 하면 연기로 인한 미세한 빛의 변화를 더 잘 감지할 수 있습니다.
		smoke2.setTiaA(0x1C36); // Slot A (Blue) TIA Gain -> 1MΩ
		smoke2.setTiaB(0x1C36); // Slot B (IR) TIA Gain -> 1MΩ
		// Blue/IR 비율이 연기 발생 시 감소하므로, 투과형 모드로 설정합니다.
		// 이렇게 하면 비율이 감소할 때 score가 증가하도록 부호가 반전됩니다.
		smoke2.setTransmissiveMode(true);
		smoke2.setPacketsToAvg(4);
		smoke2.setEmaAlpha(0.01f);
		smoke2.setWarmupSec(20);
		smoke2.setAdaptGuard(0.02f);
		smoke2.setThreshold(5000.0f); // 비현실적으로 높았던 임계값을 5000으로 수정
		smoke2.setPersist(3,5);
		smoke2.setMinIrForScaling(200000.f); // IR 스케일링 하한값을 200000으로 조정
		smoke2.setLedCurrents_mA(20.f, 20.f);
		smoke2.enableEfuseCalibration(true); // eFuse 보정 사용
		smoke2.begin(Wire, 0x64, g_config.smoke2_alpha); // 저장된 alpha 값으로 시작
		smoke2.debug_fifo_probe(Serial);
	#endif

	#if USE_SPS30
		if(!sps30.begin(Wire, 0x69)) Serial.println(F("[SPS30] init failed"));
	#endif

	#if USE_ICS43434
		if(!mic.begin(PIN_I2S_BCLK, PIN_I2S_LRCK, PIN_I2S_DIN)) Serial.println(F("[I2S] mic init failed"));
	#endif

	#if USE_ZE07
		ze07.begin(CO_SER, CO_RX_PIN, CO_TX_PIN, 9600);
		ze07.setQA(false);
		Serial.println(F("[ZE07] Warm-up recommended (few minutes)"));
	#endif

	#if USE_SEN0177
		sen0177.begin(PM_SER, PM_RX_PIN, PM_TX_PIN, 9600); // PM_SER 객체를 사용하도록 수정
	#endif

	#if USE_MQ2
		mq2cfg.v_div_ratio = 0.625f; mq2cfg.rl_ohms=5000.0f; mq2cfg.v_supply_mV=5000.0f;
		mq2cfg.warmup_s=15; mq2cfg.calib_s=20; mq2cfg.ema_alpha=0.2f; mq2cfg.alarm_thr=0.35f;
		mq2.begin(mq2cfg, g_config.mq2_r0);
		Serial.printf("[MQ2] Starting with R0 = %.2f Ohms\n", g_config.mq2_r0);
	#endif

	// 강제 설정 모드 진입을 위한 버튼 핀 설정 (내부 풀업)
	pinMode(PIN_FORCE_CONFIG_PORTAL, INPUT_PULLUP);
	// 부팅 시 버튼이 눌려있으면(LOW) 강제로 설정 포털 진입
	if (digitalRead(PIN_FORCE_CONFIG_PORTAL) == LOW) {
		startConfigPortal();
	}

	// WiFi 연결 시도
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true);
	WiFi.persistent(false);
	
	if (strlen(g_config.wifi_ssid) > 0) {
		Serial.printf("Connecting to %s", g_config.wifi_ssid);
		WiFi.begin(g_config.wifi_ssid, g_config.wifi_pass);
		uint32_t t0 = millis();
		while (WiFi.status() != WL_CONNECTED && (millis() - t0 < 15000)) {
			delay(500);
			Serial.print(".");
		}
	}

	// WiFi 연결 실패 시 설정 포털 진입
	if (WiFi.status() != WL_CONNECTED) {
		startConfigPortal(); // 이 함수는 무한 루프이므로 여기서 멈춤
	}

	String __ip = WiFi.localIP().toString();
	String __ssid = WiFi.SSID();
	String __bssid = WiFi.BSSIDstr();
	Serial.println("WiFi Connected.");
	Serial.print("ESP32 ip:");
	Serial.println(__ip);
	leds::set4(1);

	g_mqtt.setBufferSize(MAX_JSON_MSG_SIZE);
 
	//smoke2_one_shot_dump();
	Serial.println(F("{\"status\":\"ready\"}"));

	// FreeRTOS Queue 생성 (최대 5개의 메시지 저장 가능)
	g_mqttQueue = xQueueCreate(5, MAX_JSON_MSG_SIZE);
	if (g_mqttQueue == NULL) {
		Serial.println("Error creating the MQTT queue");
	}

	// FreeRTOS Task 생성
	// Core 0: 센서 데이터 수집
	xTaskCreatePinnedToCore(
		sensorTask,         /* Task 함수 */
		"SensorTask",       /* Task 이름 */
		8192,               /* Stack 크기 (JSON 처리 등으로 넉넉하게) */
		NULL,               /* Task 파라미터 */
		1,                  /* 우선순위 */
		&g_sensorTaskHandle,/* Task 핸들 */
		0);                 /* 실행 코어 */

	// Core 1: 네트워크 통신 (MQTT)
	xTaskCreatePinnedToCore(
		mqttTask,           /* Task 함수 */
		"MqttTask",         /* Task 이름 */
		4096,               /* Stack 크기 */
		NULL,               /* Task 파라미터 */
		1,                  /* 우선순위 */
		&g_mqttTaskHandle,  /* Task 핸들 */
		1);                 /* 실행 코어 */
}
 

void loop(){
	static int led_tick_count = 0;
	static bool led2 = false;

	// 100ms마다 호출되는 타이머 틱을 소모하여 LED 점멸 처리
	if(tmr100::consumeTick100ms()) {
		led_tick_count++;
		if (led_tick_count >= 5) { // 100ms * 5 = 500ms
			led_tick_count = 0;
			led2 = !led2;
			leds::blink2(led2);
		}
	}

	// 언제든지 버튼을 5초 이상 길게 눌러 설정 포털에 진입하는 기능
	if (digitalRead(PIN_FORCE_CONFIG_PORTAL) == LOW) {
		// 버튼이 눌리기 시작한 시점 기록
		if (g_buttonPressStartTime == 0) {
			g_buttonPressStartTime = millis();
		}
		// 5초 이상 눌렸는지 확인
		if (millis() - g_buttonPressStartTime >= CONFIG_PORTAL_HOLD_TIME_MS) {
			Serial.println("Button held for 5 seconds. Entering config portal by force...");
			startConfigPortal();
		}
	} else {
		// 버튼에서 손을 떼면 타이머 리셋
		g_buttonPressStartTime = 0;
	}

	// loop()는 이제 비어있거나, 매우 짧은 작업만 수행합니다.
	// 다른 모든 작업은 FreeRTOS Task에서 처리됩니다.
	// CPU가 다른 Task에게 양보하도록 짧은 delay를 줍니다.
	vTaskDelay(pdMS_TO_TICKS(1));
}
