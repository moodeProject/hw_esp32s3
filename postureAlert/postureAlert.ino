// ─────────────────────────────────────────────────────────
// 엣지(헬멧) 온디바이스 자세 이상 감지 + 경고음 + 조건부 WiFi 이벤트
// ─────────────────────────────────────────────────────────
// 설계 원칙(아키텍처 표 그대로):
//   1) 휘청거림/자세붕괴(IMU) 감지 -> 네트워크 없이 즉시 경고음 (수 ms 반응)
//   2) 심박/온습도 이상 감지 -> 경고음 + WiFi로 이벤트 전송
//   3) "권고 vs 조치필요" 판정(2개 조건 결합)도 ESP32-S3 위에서 계산.
//      WiFi는 이미 판정 끝난 결과(level)만 라즈베리파이/관리자로 보냄.
//      -> 라즈베리파이는 영상 분석 + 히트맵 + 알림 Cascade 2·3단계만 담당.
//
// posture_alert_logic.py 의 규칙을 동일하게 이식:
//   STABLE / STUMBLE(휘청거림) / COLLAPSE(자세붕괴)
//   불안정(STUMBLE or COLLAPSE) + 건강이상 -> ACTION_REQUIRED
//   불안정만 또는 건강이상만          -> RECOMMEND
//   둘 다 아님                        -> NORMAL
//
// TODO(지원님 파트 연동 지점): checkHealthAbnormal() 안의 더미를
// 실제 심박/온습도 판정 결과로 교체 -> 완료 (isVitalsAbnormal())
//
// 비콘 구역 판정: BLE 스캔은 초 단위로 블로킹되기 때문에 50Hz IMU 샘플링과
// 같은 loop()에서 돌리면 타이밍이 깨진다. 그래서 core 0에 별도 FreeRTOS
// 태스크로 분리해서 계속 스캔하게 하고, IMU/자세 판정은 원래대로 core 1의
// 기본 loop()에서 그대로 돈다. 둘 사이는 currentZone 전역 변수(뮤텍스로 보호)로
// 공유한다. 서버 SensorDataRequest DTO에 zoneId 필드가 아직 없어서, 지금은
// 서버로는 안 보내고 로컬 로그로만 확인한다.
// ─────────────────────────────────────────────────────────

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "maxSensor.h"
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ── 핀 설정 ────────────────────────────────────────────────
#define MPU_ADDR 0x68
#define SDA_PIN 5          // D4
#define SCL_PIN 6          // D5
#define BUZZER_PIN 1       // D0 (부팅/strapping 관련 없는 안전 핀)

// ── 샘플링 / 윈도우 ────────────────────────────────────────
#define SAMPLE_INTERVAL_MS 20     // 50Hz
#define POSTURE_WINDOW_SIZE 150   // 3초 (50Hz x 3s)

// ── 임계값 (1차: 실측 데이터(정상 330 / stagger 33 / collapse 25 윈도우) 기반) ──
// tilt_max, gyro_std, tilt_mean은 normal-abnormal 분포가 아직 상당히 겹쳐서
// (analyze_thresholds.py 참고) 중앙값 사이 지점으로 보수적으로 잡음 — 실착용
// 테스트하면서 오탐/미탐 보고 재조정 필수. slope는 아직 구분력이 없어서
// 원래 초기값 유지.
const float THRESH_STUMBLE_TILT_DEG   = 80.0f;
const float THRESH_STUMBLE_GYRO_STD   = 45.0f;
const float THRESH_COLLAPSE_TILT_DEG  = 40.0f;
const float THRESH_COLLAPSE_SLOPE     = 0.0f;    // deg/sample (실측 slope 중앙값이 거의 0이라 완화)

// ── WiFi / 서버 ────────────────────────────────────────────
const char* ssid      = "TODO";  // 각자 환경에 맞게 채워서 사용 (커밋 금지)
const char* password   = "TODO";
// 실제 배포된 서버 주소 + 엔드포인트. 서버(Spring)의 SensorDataController가
// "/api/sensor-data"로 열려있고, raw IMU(ax~gz)를 필수로 요구한다.
// TODO: 백엔드가 "/api/v1" 프리픽스를 실제로 붙이면 "/api/v1/sensor-data"로 변경.
// 현재는 백엔드에 프리픽스가 없어서(미반영 확인됨) /v1을 붙이면 404가 나므로
// 지금 배포된 서버에 맞춰 프리픽스 없이 둔다.
const char* sensorDataURL = "http://13.209.96.183:8080/api/sensor-data";
const char* deviceId   = "HELMET-001";
// zoneId는 서버 SensorDataRequest DTO에 아직 필드가 없어서 지금은 JSON엔 안 넣고
// Serial 로그로만 확인한다. 필드 추가되면 sendSensorData()에 포함시키면 됨.

// ── 비콘 / 구역 ────────────────────────────────────────────
// Holy-IOT 비콘 공통 UUID (FDA50693-A4E2-4FB1-AFCF-C6EB07647825), major로 구역 구분.
// major 34336/34435는 실측으로 확인된 값(테스트 위치 기준) — 실제 현장 배치되면
// zoneNameForMajor()의 매핑을 그 구역 이름으로 다시 채워야 한다.
const uint8_t TARGET_UUID[16] = {
  0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1,
  0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25
};
#define BEACON_SCAN_TIME_SEC 3
#define BEACON_STALE_MS 8000   // 이 시간 동안 새 스캔 결과가 없으면 "구역 없음" 취급

SemaphoreHandle_t zoneMutex;
uint16_t currentZoneMajor = 0;
bool currentZoneValid = false;
unsigned long currentZoneUpdatedMs = 0;

const char* zoneNameForMajor(uint16_t major) {
  switch (major) {
    case 34336: return "ZONE-01";  // 테스트 위치 기준 확인됨
    case 34435: return "ZONE-02";
    default:    return "UNKNOWN";
  }
}

// ── 상태 enum ──────────────────────────────────────────────
enum class PostureStatus { STABLE, STUMBLE, COLLAPSE };
enum class SafetyLevel   { NORMAL, RECOMMEND, ACTION_REQUIRED };

// ── 함수 프로토타입 (Arduino 자동 프로토타입 생성기가 enum class 반환형을
//    잘못 파싱해서 뒤죽박죽 만드는 문제가 있어 직접 선언해둔다) ──
void readMpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
float tiltAngleDeg(float ax, float ay, float az);
PostureStatus classifyPosture();
bool checkHealthAbnormal();
SafetyLevel determineSafetyLevel(PostureStatus posture, bool healthAbnormal);
void soundBuzzer(SafetyLevel level);
const char* levelToStr(SafetyLevel level);
const char* postureToStr(PostureStatus posture);
const char* healthOnlyLevelToStr(bool healthAbnormal);
void sendSensorData(float ax, float ay, float az, float gx, float gy, float gz);
void bleTask(void* param);

// ── 링버퍼 ─────────────────────────────────────────────────
float bufAx[POSTURE_WINDOW_SIZE], bufAy[POSTURE_WINDOW_SIZE], bufAz[POSTURE_WINDOW_SIZE];
float bufGx[POSTURE_WINDOW_SIZE], bufGy[POSTURE_WINDOW_SIZE], bufGz[POSTURE_WINDOW_SIZE];
int bufIndex = 0;
int bufCount = 0;

unsigned long lastSampleMs = 0;
float latestGyroStd = 0;

PostureStatus latestPosture = PostureStatus::STABLE;
bool latestHealthAbnormal = false;

// ═══════════════════════════════════════════════════════
// MPU6050 읽기
// ═══════════════════════════════════════════════════════
void readMpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  int16_t axRaw = Wire.read() << 8 | Wire.read();
  int16_t ayRaw = Wire.read() << 8 | Wire.read();
  int16_t azRaw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // temperature, 사용 안 함
  int16_t gxRaw = Wire.read() << 8 | Wire.read();
  int16_t gyRaw = Wire.read() << 8 | Wire.read();
  int16_t gzRaw = Wire.read() << 8 | Wire.read();

  ax = axRaw / 16384.0f * 9.8f;  // g -> m/s^2 (postureAlertLogic.py와 스케일 맞춤)
  ay = ayRaw / 16384.0f * 9.8f;
  az = azRaw / 16384.0f * 9.8f;
  gx = gxRaw / 131.0f;
  gy = gyRaw / 131.0f;
  gz = gzRaw / 131.0f;
}

// ═══════════════════════════════════════════════════════
// 자세 판정 (posture_alert_logic.py의 classify_posture 이식)
// ═══════════════════════════════════════════════════════
float tiltAngleDeg(float ax, float ay, float az) {
  float accSvm = sqrtf(ax * ax + ay * ay + az * az) + 1e-6f;
  float cosTilt = az / accSvm;
  if (cosTilt > 1.0f) cosTilt = 1.0f;
  if (cosTilt < -1.0f) cosTilt = -1.0f;
  return acosf(cosTilt) * 180.0f / PI;
}

PostureStatus classifyPosture() {
  float tilt[POSTURE_WINDOW_SIZE];
  float gyroSvm[POSTURE_WINDOW_SIZE];

  float tiltSum = 0, tiltMax = -1e9;
  float gyroSum = 0, gyroSumSq = 0;

  for (int i = 0; i < POSTURE_WINDOW_SIZE; i++) {
    tilt[i] = tiltAngleDeg(bufAx[i], bufAy[i], bufAz[i]);
    gyroSvm[i] = sqrtf(bufGx[i] * bufGx[i] + bufGy[i] * bufGy[i] + bufGz[i] * bufGz[i]);

    tiltSum += tilt[i];
    if (tilt[i] > tiltMax) tiltMax = tilt[i];
    gyroSum += gyroSvm[i];
  }

  float tiltMean = tiltSum / POSTURE_WINDOW_SIZE;
  float gyroMean = gyroSum / POSTURE_WINDOW_SIZE;

  for (int i = 0; i < POSTURE_WINDOW_SIZE; i++) {
    float d = gyroSvm[i] - gyroMean;
    gyroSumSq += d * d;
  }
  float gyroStd = sqrtf(gyroSumSq / POSTURE_WINDOW_SIZE);
  latestGyroStd = gyroStd;

  // 단순 선형회귀 기울기 (시간 t = 0..N-1)
  float n = POSTURE_WINDOW_SIZE;
  float sumT = 0, sumTT = 0, sumTTilt = 0;
  for (int i = 0; i < POSTURE_WINDOW_SIZE; i++) {
    sumT += i;
    sumTT += (float)i * i;
    sumTTilt += i * tilt[i];
  }
  float slope = (n * sumTTilt - sumT * tiltSum) / (n * sumTT - sumT * sumT);

  bool isCollapse = (tiltMean > THRESH_COLLAPSE_TILT_DEG) && (slope > THRESH_COLLAPSE_SLOPE);
  bool isStumble = (tiltMax > THRESH_STUMBLE_TILT_DEG) && (gyroStd > THRESH_STUMBLE_GYRO_STD);

  if (isCollapse) return PostureStatus::COLLAPSE;
  if (isStumble) return PostureStatus::STUMBLE;
  return PostureStatus::STABLE;
}

// ═══════════════════════════════════════════════════════
// 건강 신호 이상 여부 (지원님 파트 연동 지점 — 지금은 더미)
// ═══════════════════════════════════════════════════════
bool checkHealthAbnormal() {
  return isVitalsAbnormal();
}

// ═══════════════════════════════════════════════════════
// 4분기 결합 (posture_alert_logic.py의 determine_safety_level 이식)
// ═══════════════════════════════════════════════════════
SafetyLevel determineSafetyLevel(PostureStatus posture, bool healthAbnormal) {
  bool unstable = (posture == PostureStatus::STUMBLE || posture == PostureStatus::COLLAPSE);
  if (unstable && healthAbnormal) return SafetyLevel::ACTION_REQUIRED;
  if (unstable || healthAbnormal) return SafetyLevel::RECOMMEND;
  return SafetyLevel::NORMAL;
}

// ═══════════════════════════════════════════════════════
// 로컬 경고음 — 네트워크 없이 즉시 (설계 원칙 1번)
// ═══════════════════════════════════════════════════════
void soundBuzzer(SafetyLevel level) {
  if (level == SafetyLevel::ACTION_REQUIRED) {
    tone(BUZZER_PIN, 2000, 800);   // 긴급: 높은 톤, 길게
  } else if (level == SafetyLevel::RECOMMEND) {
    tone(BUZZER_PIN, 1000, 300);   // 권고: 짧게
  }
}

// ═══════════════════════════════════════════════════════
// 결과를 서버로 전송 (NORMAL이 아닐 때만, 상태 바뀔 때만)
// ═══════════════════════════════════════════════════════
const char* levelToStr(SafetyLevel level) {
  switch (level) {
    case SafetyLevel::ACTION_REQUIRED: return "ACTION_REQUIRED";
    case SafetyLevel::RECOMMEND: return "RECOMMEND";
    default: return "NORMAL";
  }
}

const char* postureToStr(PostureStatus posture) {
  switch (posture) {
    case PostureStatus::STUMBLE: return "STUMBLE";
    case PostureStatus::COLLAPSE: return "COLLAPSE";
    default: return "STABLE";
  }
}

// 서버(SensorDataServiceImpl)는 posture/healthAbnormal/level을 "독립적인 두 트랙"으로
// 재조합한다: level은 checkHealth()에서만 쓰이고 건강 트랙 전용으로 취급되며,
// FALLING/FALLEN 같은 추락 계열 값이 들어오면 무시하고 NORMAL로 되돌린다.
// 그래서 온보드에서 posture+health를 합쳐 만든 4단계 값을 그대로 보내면 안 되고,
// "건강 트랙만 반영한" 값으로 다시 계산해서 보내야 서버 로직과 의미가 맞는다.
const char* healthOnlyLevelToStr(bool healthAbnormal) {
  return healthAbnormal ? "RECOMMEND" : "NORMAL";
}

void sendSensorData(float ax, float ay, float az, float gx, float gy, float gz) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(sensorDataURL);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<300> doc;
  doc["deviceId"] = deviceId;
  doc["ax"] = ax; doc["ay"] = ay; doc["az"] = az;
  doc["gx"] = gx; doc["gy"] = gy; doc["gz"] = gz;
  doc["heartRate"] = (int)lastBpm;
  doc["spo2"] = (int)lastSpo2;
  doc["posture"] = postureToStr(latestPosture);
  doc["healthAbnormal"] = latestHealthAbnormal;
  doc["level"] = healthOnlyLevelToStr(latestHealthAbnormal);

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.printf("[raw 전송] 응답코드 %d\n", code);
  http.end();
}

// ═══════════════════════════════════════════════════════
// setup / loop
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BUZZER_PIN, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  Serial.println("MPU6050 초기화 완료");

  delay(200);  // MAX30102 안정화 대기
  initMaxSensor();

  WiFi.begin(ssid, password);
  Serial.print("Wi-Fi 연결 중");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nWi-Fi 연결 완료" : "\nWi-Fi 실패 (계속 진행, 버저는 동작함)");

  zoneMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(bleTask, "BeaconScan", 4096, NULL, 1, NULL, 0);
  Serial.println("비콘 스캔 태스크 시작 (core 0)");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float ax, ay, az, gx, gy, gz;
  readMpu(ax, ay, az, gx, gy, gz);

  updateVitalsBuffer(latestGyroStd);

  sendSensorData(ax, ay, az, gx, gy, gz);

  bufAx[bufIndex] = ax; bufAy[bufIndex] = ay; bufAz[bufIndex] = az;
  bufGx[bufIndex] = gx; bufGy[bufIndex] = gy; bufGz[bufIndex] = gz;
  bufIndex = (bufIndex + 1) % POSTURE_WINDOW_SIZE;
  if (bufCount < POSTURE_WINDOW_SIZE) bufCount++;

  if (bufCount < POSTURE_WINDOW_SIZE) return;  // 윈도우 아직 안 참

  PostureStatus posture = classifyPosture();
  bool healthAbnormal = checkHealthAbnormal();
  SafetyLevel level = determineSafetyLevel(posture, healthAbnormal);

  latestPosture = posture;
  latestHealthAbnormal = healthAbnormal;

  // 설계 원칙 1: 불안정 감지되면 네트워크 상관없이 즉시 버저
  if (level != SafetyLevel::NORMAL) {
    soundBuzzer(level);
  }
}

// ═══════════════════════════════════════════════════════
// 비콘 구역 판정 (core 0에서 별도 태스크로 계속 스캔)
// ═══════════════════════════════════════════════════════
bool parseIBeacon(const uint8_t* mfg, size_t len, uint16_t &major) {
  if (len < 25) return false;
  if (mfg[2] != 0x02 || mfg[3] != 0x15) return false;
  for (int i = 0; i < 16; i++) {
    if (mfg[4 + i] != TARGET_UUID[i]) return false;
  }
  major = (mfg[20] << 8) | mfg[21];
  return true;
}

class BeaconScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (!advertisedDevice.haveManufacturerData()) return;
    String md = advertisedDevice.getManufacturerData();
    if (md.length() < 25) return;

    uint16_t major;
    if (!parseIBeacon((const uint8_t*)md.c_str(), md.length(), major)) return;

    int rssi = advertisedDevice.getRSSI();

    xSemaphoreTake(zoneMutex, portMAX_DELAY);
    bool shouldUpdate = !currentZoneValid || rssi > lastBestRssi;
    if (shouldUpdate) {
      currentZoneMajor = major;
      lastBestRssi = rssi;
      currentZoneValid = true;
      currentZoneUpdatedMs = millis();
    }
    xSemaphoreGive(zoneMutex);
  }

 public:
  int lastBestRssi = -1000;
};

void bleTask(void* param) {
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  BeaconScanCallback* cb = new BeaconScanCallback();
  pBLEScan->setAdvertisedDeviceCallbacks(cb, true);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  for (;;) {
    cb->lastBestRssi = -1000;
    pBLEScan->start(BEACON_SCAN_TIME_SEC, false);
    pBLEScan->clearResults();

    // 너무 오래 갱신이 없으면(비콘 범위 밖) "구역 없음" 처리
    xSemaphoreTake(zoneMutex, portMAX_DELAY);
    if (currentZoneValid && millis() - currentZoneUpdatedMs > BEACON_STALE_MS) {
      currentZoneValid = false;
    }
    xSemaphoreGive(zoneMutex);

    vTaskDelay(pdMS_TO_TICKS(300));
  }
}
