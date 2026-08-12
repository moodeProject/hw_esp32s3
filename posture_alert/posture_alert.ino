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
// 실제 심박/온습도 판정 결과로 교체
// TODO(비콘 파트 연동 지점): ZONE_ID를 비콘 RSSI 스캔 결과로 교체
// ─────────────────────────────────────────────────────────

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

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
const char* serverURL = "http://192.168.0.10:8080/api/safety-event";
const char* deviceId   = "HELMET-001";
const char* ZONE_ID    = "TODO-ZONE";   // 비콘 파트 연동 전까지 임시값

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
void sendSafetyEvent(SafetyLevel level, PostureStatus posture, bool healthAbnormal);

// ── 링버퍼 ─────────────────────────────────────────────────
float bufAx[POSTURE_WINDOW_SIZE], bufAy[POSTURE_WINDOW_SIZE], bufAz[POSTURE_WINDOW_SIZE];
float bufGx[POSTURE_WINDOW_SIZE], bufGy[POSTURE_WINDOW_SIZE], bufGz[POSTURE_WINDOW_SIZE];
int bufIndex = 0;
int bufCount = 0;

unsigned long lastSampleMs = 0;
SafetyLevel lastSentLevel = SafetyLevel::NORMAL;  // 같은 상태 반복 전송 방지

// ═══════════════════════════════════════════════════════
// MPU6050 읽기
// ═══════════════════════════════════════════════════════
void readMpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // temperature, 사용 안 함
  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

  ax = ax_raw / 16384.0f * 9.8f;  // g -> m/s^2 (posture_alert_logic.py와 스케일 맞춤)
  ay = ay_raw / 16384.0f * 9.8f;
  az = az_raw / 16384.0f * 9.8f;
  gx = gx_raw / 131.0f;
  gy = gy_raw / 131.0f;
  gz = gz_raw / 131.0f;
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
  // TODO: 심박/온습도 판정 결과로 교체
  return false;
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

void sendSafetyEvent(SafetyLevel level, PostureStatus posture, bool healthAbnormal) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi 없음] 이벤트 전송 스킵 (버저는 이미 울림)");
    return;
  }

  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<256> doc;
  doc["deviceId"] = deviceId;
  doc["zoneId"] = ZONE_ID;
  doc["level"] = levelToStr(level);
  doc["posture"] = postureToStr(posture);
  doc["healthAbnormal"] = healthAbnormal;

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.printf("[이벤트 전송] %s (응답코드 %d)\n", body.c_str(), code);
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

  WiFi.begin(ssid, password);
  Serial.print("Wi-Fi 연결 중");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nWi-Fi 연결 완료" : "\nWi-Fi 실패 (계속 진행, 버저는 동작함)");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float ax, ay, az, gx, gy, gz;
  readMpu(ax, ay, az, gx, gy, gz);

  bufAx[bufIndex] = ax; bufAy[bufIndex] = ay; bufAz[bufIndex] = az;
  bufGx[bufIndex] = gx; bufGy[bufIndex] = gy; bufGz[bufIndex] = gz;
  bufIndex = (bufIndex + 1) % POSTURE_WINDOW_SIZE;
  if (bufCount < POSTURE_WINDOW_SIZE) bufCount++;

  if (bufCount < POSTURE_WINDOW_SIZE) return;  // 윈도우 아직 안 참

  PostureStatus posture = classifyPosture();
  bool healthAbnormal = checkHealthAbnormal();
  SafetyLevel level = determineSafetyLevel(posture, healthAbnormal);

  // 설계 원칙 1: 불안정 감지되면 네트워크 상관없이 즉시 버저
  if (level != SafetyLevel::NORMAL) {
    soundBuzzer(level);
  }

  // 상태가 "바뀌었을 때"만 서버로 전송 (매 20ms 스팸 방지)
  if (level != SafetyLevel::NORMAL && level != lastSentLevel) {
    sendSafetyEvent(level, posture, healthAbnormal);
  }
  lastSentLevel = level;
}
