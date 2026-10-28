// ─────────────────────────────────────────────────────────
// 엣지(헬멧) 온디바이스 자세 이상 감지 + 음성 안내 + 조건부 WiFi 이벤트
// ─────────────────────────────────────────────────────────
// 경고음(버저)은 MAX98357A 스피커 음성 안내로 교체됨 (voicePlayer.h, announceSituation()):
//   자세 이상만 or 건강 이상만   -> "비정상적인 징후가 감지되었습니다. 조심해주세요."
//   자세 + 건강 둘 다 이상       -> "즉시 작업을 중단하고 휴식을 취하세요."
//   낙상 후 10초 움직임 없음      -> "괜찮으시면 고개를 움직여 주세요." (본인 확인, 관리자 전송 전)
//   그 뒤 20초 더 움직임 없음     -> "의식불명으로 판별되어 관리자에게 연락을 취하고 있습니다."
//   폭염 + 고심박 + 휴식 없음     -> "열사병 위험이 있으니 휴식을 취해주세요."
//   헬멧 카메라 영상 위험 확정    -> "(띠띠띠) 전방에 위험 요소가 있습니다. 조심하세요."
//     서버가 센서 전송 응답에 hazardWarning 을 실어 준다. 영상 단독이면 관리자 대시보드에는
//     안 올라가고 이 경고로만 끝나며, 센서 이상이 겹치면 서버가 위험 이벤트로 올린다.
//
// 카메라: 같은 보드의 OV3660 을 :81/stream 으로 내보낸다 (cameraStream.h).
//   라즈베리파이가 이 스트림을 AI 서버로 중계한다. 빌드 시 PSRAM=opi 필요.
//
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
// 공유한다. 서버 SensorDataRequest DTO에 zoneId 필드가 추가되어(server:
// feat/zone-id-sensor-data) sendSensorData()에서 실제로 같이 전송한다.
// ─────────────────────────────────────────────────────────

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "maxSensor.h"
#include "fallDetector.h"   // 온보드 낙상 감지 (FastAPI /predict 이식)
#include "voicePlayer.h"    // MAX98357A 안내 음성 (기존 버저 대체)
#include "cameraStream.h"   // 헬멧 시점 MJPEG 스트림 (라즈베리파이 -> AI 서버)
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ── 핀 설정 ────────────────────────────────────────────────
#define MPU_ADDR 0x68
#define SDA_PIN 5          // D4
#define SCL_PIN 6          // D5
// 스피커(MAX98357A) 핀은 voicePlayer.h 참고 (D8/D9/D10)

// ── 샘플링 / 윈도우 ────────────────────────────────────────
#define SAMPLE_INTERVAL_MS 20     // 50Hz
#define POSTURE_WINDOW_SIZE 150   // 3초 (50Hz x 3s)

// ── 임계값 (2차: 2026-10-29 상대 기울기 재수집 — session 12~14, data_collection/analyze_posture_v2.py) ──
// 3초 윈도우(0.5초 간격) 정상 110 / 휘청 20 / 쓰러짐 10개. 기울기는 착용 기준 자세(g0) 대비.
// 휘청: 가속도 크기 변동폭(max-min)이 정상(최대 1.19g)과 휘청(최소 1.21g)을 가장 잘 갈랐고,
//       쭈그려 앉기(변동 1.19g)와 겹치지 않게 기울기 최대 60° 이상을 같이 본다. → 정상 0%, 휘청 4종 100%
//       (예전 기울기 80°+자이로 표준편차 45 는 휘청 4종 중 1종만 잡았음)
// 쓰러짐: 숙인 채로 머무는지(평균·최근 1초 기울기)와 천천히 무너지는지(회전 최대)를 같이 본다.
//       의자에 앉기(빠르게 앉음, 회전 최대 167°/s)와 물건 줍기(평균 13°)는 제외됨 → 정상 0%
// 수집량이 시나리오당 1회라 쓰러짐 데이터가 적다(벽 미끄러지기 도중 센서 접촉 끊김). 추가 수집 후 재조정.
const float THRESH_STUMBLE_ACC_RANGE_G   = 1.2f;    // 윈도우 안 가속도 크기 max-min (g)
const float THRESH_STUMBLE_TILT_MAX_DEG  = 60.0f;
const float THRESH_COLLAPSE_TILT_MEAN_DEG = 35.0f;
const float THRESH_COLLAPSE_TILT_LAST_DEG = 40.0f;  // 윈도우 마지막 1초 평균 — 지금도 숙여져 있는지
const float THRESH_COLLAPSE_GYRO_MAX_DPS  = 150.0f; // 오프셋 뺀 회전 크기 최대 — 이보다 빠르면 일부러 앉는 동작
const unsigned long POSTURE_HOLD_MS     = 1000UL;   // 이상 판정이 이만큼 이어져야 경고 (순간 튐 방지)

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
// zoneId는 sendSensorData()에서 currentZone 상태를 읽어 함께 전송한다.
// 서버 WeatherHeatRiskCache가 기상청 실황으로 계산한 체감온도 위험도
// (응답: {"data": {"apparentTemperature": .., "heatRiskLevel": "NORMAL|RECOMMEND|ACTION_REQUIRED"}})
const char* weatherURL = "http://13.209.96.183:8080/api/weather/heat-risk";

// ── 비콘 / 구역 ────────────────────────────────────────────
// Holy-IOT 비콘 공통 UUID (FDA50693-A4E2-4FB1-AFCF-C6EB07647825), major로 구역 구분.
// major 34336/34435는 실측으로 확인된 값(테스트 위치 기준) — 실제 현장 배치되면
// zoneNameForMajor()의 매핑을 그 구역 이름으로 다시 채워야 한다.
const uint8_t TARGET_UUID[16] = {
  0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1,
  0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25
};
// 실측 확인된 배포 비콘의 제조사 ID (little-endian 그대로, MFG 데이터 앞 2바이트).
const uint8_t TARGET_COMPANY_ID[2] = {0xFF, 0xFF};
#define BEACON_SCAN_TIME_SEC 3
#define BEACON_STALE_MS 8000   // 이 시간 동안 새 스캔 결과가 없으면 "구역 없음" 취급

SemaphoreHandle_t zoneMutex = NULL;
uint16_t currentZoneMajor = 0;
bool currentZoneValid = false;
unsigned long currentZoneUpdatedMs = 0;
// bleTask()에서 "구역이 실제로 바뀌었을 때만" 로그를 남기기 위한 이전 상태 기록.
uint16_t loggedZoneMajor = 0;
bool loggedZoneValid = false;

/**
 * 비콘 major 값을 사람이 읽는 구역 이름으로 변환한다.
 * @param major iBeacon major 값.
 * @return 매핑된 구역 이름, 모르는 major면 "UNKNOWN".
 */
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
bool readMpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
void resetCalibrationAccum();
void startPostureCalibration();
void updatePostureCalibration(float ax, float ay, float az, float gx, float gy, float gz);
float tiltAngleDeg(float ax, float ay, float az);
PostureStatus classifyPosture();
bool checkHealthAbnormal();
SafetyLevel determineSafetyLevel(PostureStatus posture, bool healthAbnormal);
void announceSituation(PostureStatus posture, bool healthAbnormal, unsigned long now);
bool isHeatstrokeRisk(unsigned long now);
void updateRestTracking(unsigned long now);
void updateUnconsciousWatch(float ax, float ay, float az, float gx, float gy, float gz, unsigned long now);
void weatherTask(void* param);
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

// ── 서버 전송 주기 (센서 샘플링과 분리) ──────────────────────
// [루프 타이밍 수정] 기존엔 매 루프마다 sendSensorData()를 호출해서
// HTTP 응답을 기다리는 동안 센서 샘플링(updateVitalsBuffer/자세 버퍼)까지
// 같이 멈춰버려 250개/150개 윈도우가 채워지는 데 원래 의도보다 훨씬
// 오래 걸리는 문제가 있었음. 전송만 별도 주기로 분리.
#define SEND_INTERVAL_MS 200
unsigned long lastSendMs = 0;

PostureStatus latestPosture = PostureStatus::STABLE;
bool latestHealthAbnormal = false;

// ── 착용 기준 자세(g0) 보정 ─────────────────────────────────
// 센서가 헬멧에 비스듬히 붙어 있어서 똑바로 서 있어도 절대 기울기(Z축 기준)가 ~33°로
// 나왔고, 그 탓에 고개만 조금 숙여도 COLLAPSE 조건이 걸렸다. 그래서 착용 직후 정지
// 상태 3초의 가속도 평균 방향을 기준(g0)으로 잡고, 기울기는 g0와 이루는 각도로 본다.
// 보정 전에는 g0 = (0,0,1)이라 tiltAngleDeg()가 예전 절대 기울기와 똑같이 동작한다.
// 정지 판정: 이 MPU6050은 가만히 있어도 자이로가 ~(28,-36,-7)°/s(크기 ~46°/s)로 읽히는
// 고정 오프셋이 있어서(2026-10-19 실측) 자이로 "크기"로는 정지를 못 가린다. 그래서
// 보정 중 누적한 자이로 평균에서 얼마나 벗어나는지(= 오프셋을 뺀 실제 회전)로 본다.
#define CALIB_SAMPLES 150             // 3초 @ 50Hz
const float CALIB_MAX_GYRO_DPS = 30.0f;  // 자이로 평균에서 이보다 벗어나면(움직이면) 처음부터 다시 누적
const int CALIB_MOVE_SAMPLES = 5;        // 이만큼 연달아(0.1초) 벗어나야 움직임으로 본다. 그보다 짧으면 노이즈로 버림
float g0x = 0.0f, g0y = 0.0f, g0z = 1.0f;  // 기준 단위벡터
bool postureCalibrated = false;
float calibSumX = 0, calibSumY = 0, calibSumZ = 0;
float calibGyroSumX = 0, calibGyroSumY = 0, calibGyroSumZ = 0;
int calibCount = 0;
// 보정 완료 시 저장하는 정지 상태 자이로 평균(고정 오프셋). 의식불명 판정이 이 값을 빼고
// 실제 회전만으로 움직임을 본다. 재보정 중에는 직전 값을 그대로 쓴다.
float gyroOffX = 0.0f, gyroOffY = 0.0f, gyroOffZ = 0.0f;

// ── 온보드 낙상 감지 결과 ───────────────────────────────────
// 판정은 20ms마다, 정기 전송은 200ms마다라서 그 사이에 잡힌 낙상을 놓치지 않도록
// "전송 전까지 한 번이라도 감지됐는지 / 최고 confidence"를 붙잡아 둔다.
// 낙상이 처음 잡힌 순간에는 정기 전송을 기다리지 않고 즉시 전송한다 (loop() 참고).
bool pendingFallDetected = false;
float pendingFallConfidence = 0.0f;

// ── 음성 안내 ───────────────────────────────────────────────
// 자세 판정은 정상 동작에서도 오탐이 잦아서 원래 버저에서는 빼뒀었음(위 loop() 주석).
// 음성 안내 요구사항("자세 이상만 감지 -> 1번 문장")에 맞춰 다시 반영하되,
// 오탐이 거슬리면 이 값을 false로 바꾸면 건강 이상만으로 1/2번 문장을 판단한다.
const bool VOICE_USE_POSTURE = true;
// 같은 문장은 이 간격 안에 다시 틀지 않는다 (더 심각한 문장으로 바뀌면 바로 재생).
const unsigned long VOICE_REPEAT_MS = 30000UL;
int lastVoiceClip = -1;
unsigned long lastVoiceMs = 0;

// ── 의식불명 판정: 낙상 감지 후 일정 시간 움직임이 없음 ─────────
// 낙상 직후에는 충격/뒹굴기로 자이로가 크게 튀므로 SETTLE 동안은 무시하고,
// 그 뒤 자이로 크기(보정 때 잰 오프셋을 뺀 값)가 MOVE_DPS를 한 번도 넘지 않으면
// CHECK 후 본인 확인 안내, 다시 CONFIRM 동안 조용하면 의식불명으로 확정한다.
// 보정 전에는 오프셋이 0이라 정지해도 움직임으로 잡힐 수 있다.
// 값은 아직 실측 전 — 시연 시나리오(헬멧 내려놓기 등) 돌려보며 조정 필요.
// 시연 때는 CHECK/CONFIRM 을 줄여서 쓰면 된다 (현장 기준이면 더 길게).
const unsigned long UNCONSCIOUS_SETTLE_MS  = 2000UL;
const unsigned long UNCONSCIOUS_CHECK_MS   = 10000UL;  // 이만큼 조용하면 본인 확인 안내
const unsigned long UNCONSCIOUS_CONFIRM_MS = 20000UL;  // 안내 후 이만큼 더 조용하면 의식불명 확정
const float UNCONSCIOUS_MOVE_DPS = 30.0f;
bool unconsciousWatching = false;   // 낙상 후 움직임 감시 중
unsigned long unconsciousWatchStartMs = 0;
bool unconsciousCheckPrompted = false;  // 본인 확인 안내를 했고 응답(움직임)을 기다리는 중
unsigned long unconsciousCheckMs = 0;
bool unconscious = false;           // 의식불명 확정 (움직임 생기면 해제)

// ── 열사병 판정: 폭염(서버 기상) + 심박 상승 + 쉬지 않고 계속 활동 ─────
// maxSensor.h의 isHeatRiskAbnormal()은 "활동량 낮은데 심박 높음"(몸 이상 신호)이고,
// 여기는 "더운 날 심박이 높은데 휴식 없이 계속 일함"이라 의미가 달라서 따로 판정한다.
// 휴식 = 자이로 std가 HEAT_LOW_ACTIVITY_GYRO_STD 미만인 상태가 REST_MIN_MS 이상 지속.
const unsigned long REST_MIN_MS = 60000UL;            // 1분 이상 가만히 있으면 쉰 것으로 인정
const unsigned long NO_REST_LIMIT_MS = 1200000UL;     // 20분 넘게 휴식 없으면 위험 (시연 시 단축)
const float HEAT_HR_FALLBACK_BPM = 110.0f;            // 개인 기준선 확정 전에 쓰는 고심박 기준
unsigned long restCandidateStartMs = 0;
unsigned long lastRestMs = 0;
volatile bool weatherHot = false;   // weatherTask가 갱신 (heatRiskLevel >= RECOMMEND)
volatile unsigned long weatherOkMs = 0;  // 마지막으로 기상 조회에 성공한 시각
// 조회가 계속 실패하면 직전 폭염 값을 이 시간까지만 믿는다 (짧은 네트워크 장애는 버팀)
#define WEATHER_STALE_MS 1800000UL
// 심박이 이 시간 넘게 갱신되지 않으면(미착용·모션) 직전 고심박으로 판정하지 않는다
#define HEART_STALE_MS 30000UL
#define WEATHER_REFRESH_MS 600000UL  // 서버 캐시도 10분 주기라 더 자주 볼 필요 없음
#define WEATHER_RETRY_MS 30000UL

// ═══════════════════════════════════════════════════════
// MPU6050 읽기
// ═══════════════════════════════════════════════════════
/** 14바이트를 다 못 읽으면 false. 이때 값은 0xFF 등 쓰레기라 판정에 넣으면 안 된다. */
bool readMpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;

  int16_t axRaw = Wire.read() << 8 | Wire.read();
  int16_t ayRaw = Wire.read() << 8 | Wire.read();
  int16_t azRaw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // temperature, 사용 안 함
  int16_t gxRaw = Wire.read() << 8 | Wire.read();
  int16_t gyRaw = Wire.read() << 8 | Wire.read();
  int16_t gzRaw = Wire.read() << 8 | Wire.read();

  ax = axRaw / 2048.0f * 9.8f;  // g -> m/s^2 (±16g 기준: 2048 = 1g)
  ay = ayRaw / 2048.0f * 9.8f;
  az = azRaw / 2048.0f * 9.8f;
  gx = gxRaw / 16.4f;           // deg/s (±2000°/s 기준: 16.4 = 1°/s)
  gy = gyRaw / 16.4f;
  gz = gzRaw / 16.4f;
  return true;
}

// ═══════════════════════════════════════════════════════
// 자세 판정 (posture_alert_logic.py의 classify_posture 이식)
// ═══════════════════════════════════════════════════════
/**
 * 기준 자세 보정을 (다시) 시작한다. 누적을 비우고 자세 경고를 끈다.
 * 보정이 끝날 때까지(정지 3초) 자세 판정은 STABLE로만 나간다.
 */
void resetCalibrationAccum() {
  calibSumX = calibSumY = calibSumZ = 0;
  calibGyroSumX = calibGyroSumY = calibGyroSumZ = 0;
  calibCount = 0;
}

void startPostureCalibration() {
  postureCalibrated = false;
  resetCalibrationAccum();
  Serial.println("[자세] 기준 자세 보정 시작 — 정면을 보고 3초간 서 주세요");
  // TODO: voiceClips.h에 "정면을 보고 3초간 서 주세요" 클립이 없음. voice_tools/make_voice_clips.py
  // CLIPS에 추가해서 다시 생성한 뒤 requestVoice(VoiceClip::CALIB_STAND) 같은 식으로 재생.
}

/**
 * 매 샘플 호출. 정지 샘플을 CALIB_SAMPLES개 모으면 평균 가속도 방향을 g0로 저장한다.
 * 지금 자이로가 누적 중인 자이로 평균에서 CALIB_MAX_GYRO_DPS 넘게 벗어나면(움직이면)
 * 이번 샘플부터 새로 누적한다 (고정 오프셋은 평균에 들어가 있으므로 정지로 본다).
 */
void updatePostureCalibration(float ax, float ay, float az, float gx, float gy, float gz) {
  if (postureCalibrated) return;

  // 정지 중에도 I2C 노이즈로 한 샘플씩 튀는 값(-0.1,-0.1,-0.1 등)이 섞여 들어와서, 튄 샘플 하나로
  // 처음부터 다시 세면 보정이 영영 안 끝난다. 그래서 튄 샘플은 버리고, 연달아 벗어날 때만 움직임으로 본다.
  static int calibOutlierRun = 0;
  if (calibCount > 0) {
    float dx = gx - calibGyroSumX / calibCount;
    float dy = gy - calibGyroSumY / calibCount;
    float dz = gz - calibGyroSumZ / calibCount;
    if (sqrtf(dx * dx + dy * dy + dz * dz) > CALIB_MAX_GYRO_DPS) {
      if (++calibOutlierRun < CALIB_MOVE_SAMPLES) return;  // 한두 번 튄 값 — 버린다
      resetCalibrationAccum();                              // 계속 벗어남 — 실제로 움직이는 중
    }
  }
  calibOutlierRun = 0;

  calibSumX += ax; calibSumY += ay; calibSumZ += az;
  calibGyroSumX += gx; calibGyroSumY += gy; calibGyroSumZ += gz;
  calibCount++;
  if (calibCount < CALIB_SAMPLES) return;

  float norm = sqrtf(calibSumX * calibSumX + calibSumY * calibSumY + calibSumZ * calibSumZ);
  if (norm < 1e-6f) {   // 센서 값이 비정상(전부 0 등) — 다시 모은다
    resetCalibrationAccum();
    return;
  }
  g0x = calibSumX / norm;
  g0y = calibSumY / norm;
  g0z = calibSumZ / norm;
  gyroOffX = calibGyroSumX / calibCount;
  gyroOffY = calibGyroSumY / calibCount;
  gyroOffZ = calibGyroSumZ / calibCount;
  postureCalibrated = true;
  Serial.printf("[자세] 기준 자세 저장 완료 (g0=%.3f, %.3f, %.3f, 정지 시 자이로 오프셋=%.1f, %.1f, %.1f °/s)\n",
                g0x, g0y, g0z, gyroOffX, gyroOffY, gyroOffZ);
  // 보정이 끝났는지 겉으로 알 수 없어서, 판정이 시작됐다는 걸 소리로 알린다
  requestVoice(VoiceClip::CALIB_DONE);
}

static float med3(float a, float b, float c) {
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

/** 6축 각각 최근 3샘플(이번 포함)의 중간값으로 바꾼다. 처음 두 샘플은 그대로 통과. */
void medianFilter3(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  static float hist[2][6];
  static int seen = 0;
  float cur[6] = {ax, ay, az, gx, gy, gz};
  float out[6];
  for (int k = 0; k < 6; k++) out[k] = seen >= 2 ? med3(hist[0][k], hist[1][k], cur[k]) : cur[k];
  for (int k = 0; k < 6; k++) { hist[0][k] = hist[1][k]; hist[1][k] = cur[k]; }
  if (seen < 2) seen++;
  ax = out[0]; ay = out[1]; az = out[2]; gx = out[3]; gy = out[4]; gz = out[5];
}

/** 현재 가속도 벡터와 기준 자세 g0 사이의 각도(도). 보정 전에는 g0=(0,0,1)이라 절대 기울기. */
float tiltAngleDeg(float ax, float ay, float az) {
  float accSvm = sqrtf(ax * ax + ay * ay + az * az) + 1e-6f;
  float cosTilt = (ax * g0x + ay * g0y + az * g0z) / accSvm;
  if (cosTilt > 1.0f) cosTilt = 1.0f;
  if (cosTilt < -1.0f) cosTilt = -1.0f;
  return acosf(cosTilt) * 57.2958f;
}

PostureStatus classifyPosture() {
  float tilt[POSTURE_WINDOW_SIZE];
  float gyroSvm[POSTURE_WINDOW_SIZE];

  float tiltSum = 0, tiltMax = -1e9;
  float gyroSum = 0, gyroSumSq = 0;

  // 링버퍼라 물리 인덱스 0..N-1 은 시간순이 아니다. 윈도우가 찬 뒤 bufIndex 는 다음에 덮어쓸
  // 자리 = 가장 오래된 샘플이므로 거기서부터 읽어 tilt[] 를 시간순으로 채운다 (slope 회귀용).
  for (int i = 0; i < POSTURE_WINDOW_SIZE; i++) {
    int idx = (bufIndex + i) % POSTURE_WINDOW_SIZE;
    tilt[i] = tiltAngleDeg(bufAx[idx], bufAy[idx], bufAz[idx]);
    gyroSvm[i] = sqrtf(bufGx[idx] * bufGx[idx] + bufGy[idx] * bufGy[idx] + bufGz[idx] * bufGz[idx]);

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
  latestGyroStd = gyroStd;   // 건강/휴식 판정도 쓰는 값이라 보정 여부와 무관하게 갱신

  // 기준 자세가 없으면 기울기를 믿을 수 없으므로 자세 경고는 내지 않는다
  if (!postureCalibrated) return PostureStatus::STABLE;

  // 가속도 크기 변동폭(g), 마지막 1초 기울기 평균, 오프셋 뺀 회전 크기 최대
  const int LAST_N = 50;  // 1초 @ 50Hz
  float accMin = 1e9, accMax = -1e9, tiltLastSum = 0, gyroRotMax = 0;
  for (int i = 0; i < POSTURE_WINDOW_SIZE; i++) {
    int idx = (bufIndex + i) % POSTURE_WINDOW_SIZE;
    float acc = sqrtf(bufAx[idx] * bufAx[idx] + bufAy[idx] * bufAy[idx] + bufAz[idx] * bufAz[idx]) / 9.8f;
    if (acc < accMin) accMin = acc;
    if (acc > accMax) accMax = acc;
    if (i >= POSTURE_WINDOW_SIZE - LAST_N) tiltLastSum += tilt[i];
    float dx = bufGx[idx] - gyroOffX, dy = bufGy[idx] - gyroOffY, dz = bufGz[idx] - gyroOffZ;
    float rot = sqrtf(dx * dx + dy * dy + dz * dz);
    if (rot > gyroRotMax) gyroRotMax = rot;
  }
  float accRange = accMax - accMin;
  float tiltLast = tiltLastSum / LAST_N;

  bool isCollapse = (tiltMean > THRESH_COLLAPSE_TILT_MEAN_DEG) && (tiltLast > THRESH_COLLAPSE_TILT_LAST_DEG) &&
                    (gyroRotMax < THRESH_COLLAPSE_GYRO_MAX_DPS);
  bool isStumble = (accRange >= THRESH_STUMBLE_ACC_RANGE_G) && (tiltMax >= THRESH_STUMBLE_TILT_MAX_DEG);

  PostureStatus raw = isCollapse ? PostureStatus::COLLAPSE
                    : isStumble  ? PostureStatus::STUMBLE
                                 : PostureStatus::STABLE;

  // 이상 판정이 POSTURE_HOLD_MS 동안 계속 이어질 때만 내보낸다 (한 순간 걸쳤다 빠지는 경우 제외).
  static unsigned long unstableSinceMs = 0;
  if (raw == PostureStatus::STABLE) {
    unstableSinceMs = 0;
    return PostureStatus::STABLE;
  }
  unsigned long now = millis();
  if (unstableSinceMs == 0) unstableSinceMs = now;
  if (now - unstableSinceMs < POSTURE_HOLD_MS) return PostureStatus::STABLE;
  return raw;
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
// 로컬 음성 안내 — 네트워크 없이 즉시 (설계 원칙 1번, 기존 버저 대체)
// ═══════════════════════════════════════════════════════
/**
 * 지금 상황에 맞는 안내 문장을 골라 재생 요청한다. 우선순위(위가 더 심각):
 *   의식불명 > 자세+건강 동시 이상 > 열사병 위험 > 자세 or 건강 단독 이상
 * 같은 문장은 VOICE_REPEAT_MS 안에 반복하지 않고, 정상으로 돌아오면 기록을 지워서
 * 다음 이상이 생기면 바로 안내한다.
 */
void announceSituation(PostureStatus posture, bool healthAbnormal, unsigned long now) {
  // 본인 확인 안내 후 응답을 기다리는 동안은 다른 안내로 덮어쓰지 않는다
  if (unconsciousCheckPrompted) return;

  bool unstable = VOICE_USE_POSTURE && postureCalibrated &&
                  (posture == PostureStatus::STUMBLE || posture == PostureStatus::COLLAPSE);

  int clip = -1;
  if (unconscious) {
    clip = (int)VoiceClip::UNCONSCIOUS;
  } else if (unstable && healthAbnormal) {
    clip = (int)VoiceClip::STOP_AND_REST;
  } else if (isHeatstrokeRisk(now)) {
    clip = (int)VoiceClip::HEATSTROKE;
  } else if (unstable || healthAbnormal) {
    clip = (int)VoiceClip::WARN_ABNORMAL;
  }

  if (clip < 0) {
    lastVoiceClip = -1;
    return;
  }
  // VoiceClip enum은 심각도 순서가 아니라서 비교 대신 "다른 문장이면 바로 재생"으로 처리
  if (clip == lastVoiceClip && now - lastVoiceMs < VOICE_REPEAT_MS) return;

  requestVoice((VoiceClip)clip);
  lastVoiceClip = clip;
  lastVoiceMs = now;
}

/** 폭염 + 심박 상승 + 휴식 없이 NO_REST_LIMIT_MS 이상 활동 중이면 true. */
bool isHeatstrokeRisk(unsigned long now) {
  if (!weatherHot || weatherOkMs == 0 || now - weatherOkMs > WEATHER_STALE_MS) return false;
  if (lastBpm <= 0 || lastBpmMs == 0 || now - lastBpmMs > HEART_STALE_MS) return false;
  float hrLimit = baselineReady ? baselineHrMean + HEAT_HR_MARGIN : HEAT_HR_FALLBACK_BPM;
  if (lastBpm <= hrLimit) return false;
  return now - lastRestMs >= NO_REST_LIMIT_MS;
}

/** 자이로 std가 낮은 상태가 REST_MIN_MS 이상 이어지면 "쉬었다"로 기록한다. 매 샘플 호출. */
void updateRestTracking(unsigned long now) {
  if (latestGyroStd < HEAT_LOW_ACTIVITY_GYRO_STD) {
    if (restCandidateStartMs == 0) restCandidateStartMs = now;
    if (now - restCandidateStartMs >= REST_MIN_MS) lastRestMs = now;
  } else {
    restCandidateStartMs = 0;
  }
}

/**
 * 낙상 후 움직임 감시. 매 샘플 호출. 2단계로 확정한다:
 *   1) SETTLE 이후 CHECK 동안 움직임이 없으면 본인 확인 안내만 한다 (관리자에게는 아직 안 보냄)
 *   2) 안내 후 CONFIRM 동안에도 움직임이 없으면 의식불명 확정 + 서버로 즉시 전송
 * 어느 단계든 움직임이 잡히면(고개 끄덕이기 등) 감시를 끝낸다. 의식불명 상태에서 움직이면 해제.
 * 낙상 자체는 감지 순간 이미 서버로 갔으므로 이 감시와 무관하게 관리자 화면에 뜬다.
 */
void updateUnconsciousWatch(float ax, float ay, float az, float gx, float gy, float gz, unsigned long now) {
  // 고정 오프셋(~46°/s)을 빼지 않으면 가만히 있어도 MOVE_DPS(30)를 넘어 의식불명이 확정되지 않는다
  float dx = gx - gyroOffX, dy = gy - gyroOffY, dz = gz - gyroOffZ;
  float gyroMag = sqrtf(dx * dx + dy * dy + dz * dz);
  bool moving = gyroMag > UNCONSCIOUS_MOVE_DPS;

  if (unconscious) {
    if (moving) {
      unconscious = false;
      Serial.println("[의식불명] 움직임 감지 — 해제");
    }
    return;
  }
  if (!unconsciousWatching) return;

  unsigned long elapsed = now - unconsciousWatchStartMs;
  if (elapsed < UNCONSCIOUS_SETTLE_MS) return;
  if (moving) {
    unconsciousWatching = false;
    Serial.println(unconsciousCheckPrompted ? "[의식불명 감시] 본인 확인 응답(움직임) — 감시 종료"
                                            : "[의식불명 감시] 움직임 있음 — 감시 종료");
    unconsciousCheckPrompted = false;
    return;
  }
  if (!unconsciousCheckPrompted) {
    if (elapsed >= UNCONSCIOUS_SETTLE_MS + UNCONSCIOUS_CHECK_MS) {
      unconsciousCheckPrompted = true;
      unconsciousCheckMs = now;
      Serial.println("[의식불명 감시] 움직임 없음 — 본인 확인 안내");
      requestVoice(VoiceClip::CHECK_RESPONSE);
      lastVoiceClip = (int)VoiceClip::CHECK_RESPONSE;
      lastVoiceMs = now;
    }
    return;
  }
  if (now - unconsciousCheckMs >= UNCONSCIOUS_CONFIRM_MS) {
    unconsciousWatching = false;
    unconsciousCheckPrompted = false;
    unconscious = true;
    Serial.println("[의식불명] 본인 확인 안내 후에도 움직임 없음 — 관리자 알림 전송");
    // sendSensorData()가 unconscious=true 를 실어 보내면 서버가 UNCONSCIOUS 위험 이벤트를 만든다.
    lastSendMs = now;
    sendSensorData(ax, ay, az, gx, gy, gz);
    requestVoice(VoiceClip::UNCONSCIOUS);  // 자세 윈도우(3초) 차기 전이라도 바로 안내
    lastVoiceClip = (int)VoiceClip::UNCONSCIOUS;
    lastVoiceMs = now;
  }
}

/**
 * core 0 태스크. 서버의 체감온도 위험도를 주기적으로 받아 weatherHot을 갱신한다.
 * loop()에서 GET을 하면 응답 대기 동안 50Hz 샘플링이 멈추므로 분리했다.
 * 실패하면 직전 값을 유지하고 WEATHER_RETRY_MS 뒤 재시도.
 */
void weatherTask(void* param) {
  for (;;) {
    bool ok = false;
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(weatherURL);
      int code = http.GET();
      if (code == 200) {
        StaticJsonDocument<256> doc;
        if (!deserializeJson(doc, http.getString())) {
          const char* level = doc["data"]["heatRiskLevel"] | "NORMAL";
          weatherHot = strcmp(level, "RECOMMEND") == 0 || strcmp(level, "ACTION_REQUIRED") == 0;
          float apparent = doc["data"]["apparentTemperature"] | -999.0f;
          Serial.printf("[기상] 체감온도 %.1f, level=%s -> 폭염=%d\n", apparent, level, weatherHot);
          weatherOkMs = millis();
          ok = true;
        }
      }
      if (!ok) Serial.printf("[기상] 조회 실패 (응답코드 %d) — 직전 값 유지\n", code);
      http.end();
    }
    vTaskDelay(pdMS_TO_TICKS(ok ? WEATHER_REFRESH_MS : WEATHER_RETRY_MS));
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

// ── 서버 전송 전용 태스크 (loop가 HTTP 응답을 기다리느라 멈추지 않게) ──
// loop(판정)는 보낼 내용을 "우편함(sendQueue)"에 넣기만 하고 바로 다음 판정으로 넘어가고,
// 실제 HTTP 전송과 응답 대기는 core 0에서 따로 도는 sendTask가 맡는다.
struct SendMsg {
  char body[512];   // 서버로 보낼 JSON 문자열
  bool hadFall;     // 이 메시지에 낙상=true가 실렸는지
};
QueueHandle_t sendQueue = NULL;        // 한 칸짜리 우편함 (안 나간 메시지는 최신 것으로 덮어씀)
volatile bool fallDelivered = false;   // 낙상이 실린 메시지를 서버가 받았으면 true
long lastHazardWarningSeq = 0;         // 마지막으로 안내한 영상 경고 번호 (sendTask 전용)

/**
 * 서버 응답의 영상 경고(data.hazardWarningSeq)를 보고, 새 번호일 때만 경고 음성을 낸다.
 * 서버는 경고를 10초 동안 매 응답에 실어 주므로 번호로 중복을 거른다.
 */
void handleHazardWarning(const String& body) {
  StaticJsonDocument<64> filter;
  filter["data"]["hazardWarningSeq"] = true;
  filter["data"]["hazardWarning"] = true;
  StaticJsonDocument<192> res;
  if (deserializeJson(res, body, DeserializationOption::Filter(filter)) != DeserializationError::Ok) return;

  long seq = res["data"]["hazardWarningSeq"] | 0L;
  if (seq == 0 || seq == lastHazardWarningSeq) return;
  lastHazardWarningSeq = seq;
  Serial.printf("[영상 경고] %s (seq=%ld)\n", (const char*)(res["data"]["hazardWarning"] | "?"), seq);
  requestVoice(VoiceClip::HAZARD_AHEAD);
}

void sendTask(void* param) {
  SendMsg msg;
  HTTPClient http;
  for (;;) {
    if (xQueueReceive(sendQueue, &msg, portMAX_DELAY) != pdTRUE) continue;
    if (WiFi.status() != WL_CONNECTED) continue;
    http.begin(sensorDataURL);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST((uint8_t*)msg.body, strlen(msg.body));
    Serial.printf("[raw 전송] 응답코드 %d\n", code);
    if (code > 0 && msg.hadFall) fallDelivered = true;
    if (code == 200) handleHazardWarning(http.getString());
    http.end();
  }
}

void sendSensorData(float ax, float ay, float az, float gx, float gy, float gz) {
  if (sendQueue == NULL) return;

  // 낙상이 서버에 전달된 게 확인되면 그때 낙상 표시를 내림 (실패하면 다음 메시지에 다시 실림)
  if (fallDelivered) {
    pendingFallDetected = false;
    pendingFallConfidence = 0.0f;
    fallDelivered = false;
  }

  StaticJsonDocument<512> doc;   // 400 -> 512 (fallDetected/fallConfidence 추가로 여유 확보)
  doc["deviceId"] = deviceId;
  doc["ax"] = ax; doc["ay"] = ay; doc["az"] = az;
  doc["gx"] = gx; doc["gy"] = gy; doc["gz"] = gz;
  doc["heartRate"] = (int)lastBpm;
  doc["spo2"] = (int)lastSpo2;
  doc["posture"] = postureToStr(latestPosture);
  doc["healthAbnormal"] = latestHealthAbnormal;
  doc["level"] = healthOnlyLevelToStr(latestHealthAbnormal);
  // 지원님 파트: 원인별 값 (서버에 원인 구분해서 저장/노출하기 위함)
  doc["hrv"] = getLastHrv();
  doc["fatigueAbnormal"] = isFatigueAbnormal();
  doc["heatRiskAbnormal"] = isHeatRiskAbnormal();
  // 온보드 낙상 판정 결과 (기존 FastAPI /predict 응답의 fallDetected / confidence 자리)
  doc["fallDetected"] = pendingFallDetected;
  doc["fallConfidence"] = pendingFallConfidence;
  // 의식불명 확정 상태 (서버가 UNCONSCIOUS 추락 상태 + 위험 이벤트로 처리). 움직이면 false 로 돌아감.
  doc["unconscious"] = unconscious;

  // 서버 SensorDataRequest에 zoneId 필드가 추가되어(server PR: feat/zone-id-sensor-data)
  // 이제 실제로 같이 보낸다. 유효한 구역이 없으면 zoneId 필드 자체를 넣지 않는다
  // (null로 명시하는 게 아니라 키 자체가 JSON에서 생략됨).
  // zoneMutex는 비콘 태스크 초기화 실패 시 NULL일 수 있으므로(setup() 참고) 먼저 확인한다.
  bool zoneValid = false;
  uint16_t zoneMajor = 0;
  if (zoneMutex != NULL) {
    xSemaphoreTake(zoneMutex, portMAX_DELAY);
    zoneValid = currentZoneValid;
    zoneMajor = currentZoneMajor;
    xSemaphoreGive(zoneMutex);
  }
  if (zoneValid) {
    doc["zoneId"] = zoneNameForMajor(zoneMajor);
  }

  SendMsg msg;
  msg.hadFall = pendingFallDetected;
  serializeJson(doc, msg.body, sizeof(msg.body));
  xQueueOverwrite(sendQueue, &msg);   // 우편함에 넣고 바로 리턴 (응답 기다리지 않음)

  // 낙상이 아닐 때만 확률 초기화 (확률 = 직전 전송 이후 최고값)
  if (!pendingFallDetected) pendingFallConfidence = 0.0f;
}

/**
 * 부팅 때 센서 I2C 버스(SDA=D4, SCL=D5)에 붙은 장치를 시리얼 모니터에 찍는다.
 * 센서가 "인식 안 됨"일 때 배선 문제인지(아무것도 없음) 모듈 하나 문제인지(하나만 없음) 바로 구분하려고.
 */
void logI2cDevices() {
  bool mpuFound = false, maxFound = false;
  Serial.print("[I2C] 발견된 장치:");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;
    Serial.printf(" 0x%02X", addr);
    if (addr == MPU_ADDR) mpuFound = true;
    if (addr == MAX30102_ADDR) maxFound = true;
  }
  Serial.println();
  Serial.printf("[I2C] MPU6050(0x68): %s / MAX30102(0x57): %s\n",
                mpuFound ? "인식됨" : "없음", maxFound ? "인식됨" : "없음");
}

// ═══════════════════════════════════════════════════════
// setup / loop
// ═══════════════════════════════════════════════════════
/**
 * 센서(MPU6050, MAX30102), WiFi, 비콘 스캔 태스크를 초기화한다.
 * 뮤텍스/태스크 생성에 실패하면 비콘 구역 판정만 비활성화하고 나머지는 계속 진행한다.
 */
void setup() {
  Serial.begin(115200);
  delay(500);

  initVoicePlayer();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  // 측정 범위 확대 — 기본값(±2g/±250°/s)은 낙상 충격·회전이 잘려서 판정이 약해짐
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);   // 가속도 범위 설정 레지스터
  Wire.write(0x18);   // ±16g
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);   // 자이로 범위 설정 레지스터
  Wire.write(0x18);   // ±2000°/s
  Wire.endTransmission();
  Serial.println("MPU6050 초기화 완료");

  delay(200);
  logI2cDevices();
  initMaxSensor();

  WiFi.begin(ssid, password);
  Serial.print("Wi-Fi 연결 중");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nWi-Fi 연결 완료" : "\nWi-Fi 실패 (계속 진행, 음성 안내는 동작함)");
  WiFi.setSleep(false);  // 절전 모드면 카메라 스트림이 끊기듯 느려진다
  startCameraStream();

  lastRestMs = millis();  // 착용 시작 시점을 마지막 휴식으로 간주
  if (xTaskCreatePinnedToCore(weatherTask, "Weather", 8192, NULL, 1, NULL, 0) != pdPASS) {
    Serial.println("[기상] 조회 태스크 생성 실패 — 열사병 안내 비활성화");
  }

  zoneMutex = xSemaphoreCreateMutex();
  if (zoneMutex == NULL) {
    Serial.println("[비콘] 뮤텍스 생성 실패 — 비콘 구역 판정 비활성화");
  } else {
    BaseType_t taskCreated = xTaskCreatePinnedToCore(bleTask, "BeaconScan", 4096, NULL, 1, NULL, 0);
    if (taskCreated != pdPASS) {
      Serial.println("[비콘] 스캔 태스크 생성 실패 — 비콘 구역 판정 비활성화");
      vSemaphoreDelete(zoneMutex);
      zoneMutex = NULL;
    } else {
      Serial.println("비콘 스캔 태스크 시작 (core 0)");
    }
  }

  sendQueue = xQueueCreate(1, sizeof(SendMsg));
  if (sendQueue == NULL ||
      xTaskCreatePinnedToCore(sendTask, "SensorSend", 8192, NULL, 1, NULL, 0) != pdPASS) {
    Serial.println("[전송] 태스크 생성 실패 — 서버 전송 비활성화");
    sendQueue = NULL;
  } else {
    Serial.println("서버 전송 태스크 시작 (core 0)");
  }

  startPostureCalibration();  // 착용 직후 정지 3초로 기준 자세 저장
}

void loop() {
  // [테스트용] 시리얼에 1~5 입력 시 해당 안내 음성 바로 재생 (스피커 동작 확인)
  //           'c' 입력 시 기준 자세 재보정 (다시 정지 3초)
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '5') requestVoice((VoiceClip)(c - '1'));
    else if (c == 'c') startPostureCalibration();
    else if (c == '+') { fallThreshold = min(0.95f, fallThreshold + 0.05f); Serial.printf("[낙상 기준값] %.2f\n", fallThreshold); }
    else if (c == '-') { fallThreshold = max(0.05f, fallThreshold - 0.05f); Serial.printf("[낙상 기준값] %.2f\n", fallThreshold); }
  }

  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float ax, ay, az, gx, gy, gz;
  // I2C 읽기 실패 샘플은 버린다. 섞이면 정지 상태도 움직임으로 보여 기준 자세 저장이 끝나지 않는다.
  if (!readMpu(ax, ay, az, gx, gy, gz)) return;
  updatePostureCalibration(ax, ay, az, gx, gy, gz);

  // [디버그] 1초마다 현재 상대 tilt 출력 — 정자세 0.1~1.6° 확인됨(2026-10-19), 필요할 때 주석 해제
  // static unsigned long tiltDbgMs = 0;
  // if (postureCalibrated && now - tiltDbgMs >= 1000) {
  //   Serial.printf("[자세 디버그] 상대 tilt=%.1f°\n", tiltAngleDeg(ax, ay, az));
  //   tiltDbgMs = now;
  // }

  updateVitalsBuffer(latestGyroStd);  // MAX30102 활성화/비활성화

  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendSensorData(ax, ay, az, gx, gy, gz);
  }

  // 정지 중에도 I2C 노이즈로 한 샘플만 튀는 값(기울기 137°, 자이로 수백 °/s)이 섞여 들어와
  // 그 한 개로 휘청거림이 판정됐다. 최근 3개 중간값을 넣어 한 샘플짜리 튐만 지운다(지연 20ms).
  // 실제 움직임은 여러 샘플에 걸쳐 이어지므로 그대로 남는다.
  medianFilter3(ax, ay, az, gx, gy, gz);
  bufAx[bufIndex] = ax; bufAy[bufIndex] = ay; bufAz[bufIndex] = az;
  bufGx[bufIndex] = gx; bufGy[bufIndex] = gy; bufGz[bufIndex] = gz;
  bufIndex = (bufIndex + 1) % POSTURE_WINDOW_SIZE;
  if (bufCount < POSTURE_WINDOW_SIZE) bufCount++;

  // ── 온보드 낙상 판정 (FastAPI /predict와 동일: 최근 50개 윈도우) ──
  // 자세 판정(150개)보다 먼저 돌려야 버퍼 50개만 차도 바로 판정 가능
  if (bufCount >= FALL_WINDOW_SIZE) {
    static float fallWindow[FALL_WINDOW_SIZE][6];
    for (int i = 0; i < FALL_WINDOW_SIZE; i++) {
      int idx = (bufIndex - FALL_WINDOW_SIZE + i + POSTURE_WINDOW_SIZE) % POSTURE_WINDOW_SIZE;
      fallWindow[i][0] = bufAx[idx]; fallWindow[i][1] = bufAy[idx]; fallWindow[i][2] = bufAz[idx];
      fallWindow[i][3] = bufGx[idx]; fallWindow[i][4] = bufGy[idx]; fallWindow[i][5] = bufGz[idx];
    }
    FallResult fall = predictFall(fallWindow);

        // [임시 디버그] 1초마다 판정 횟수와 최고 confidence 출력 - 테스트 후 삭제
    static float dbgMax = 0; static int dbgCnt = 0; static unsigned long dbgMs = 0;
    if (fall.confidence > dbgMax) dbgMax = fall.confidence;
    dbgCnt++;
    if (now - dbgMs >= 1000) {
      Serial.printf("[디버그] 1초간 판정 %d회, 최고 conf=%.2f\n", dbgCnt, dbgMax);
      dbgMax = 0; dbgCnt = 0; dbgMs = now;
    }

    if (fall.confidence > pendingFallConfidence) pendingFallConfidence = fall.confidence;
    // prevFallDetected: 20ms 전 판정 결과. 정상 -> 낙상으로 "바뀐 순간"에만 즉시 전송해서
    // 충격이 윈도우에 남아있는 ~1초 동안 전송이 20ms마다 반복되는 것을 막는다.
    static bool prevFallDetected = false;
    if (fall.fallDetected) {
      pendingFallDetected = true;                  // 다음 정기 전송에도 낙상으로 실려감
      if (!prevFallDetected) {
        Serial.printf("[낙상 감지] confidence=%.2f\n", fall.confidence);
        lastSendMs = now;                          // 정기 전송 타이머 리셋
        sendSensorData(ax, ay, az, gx, gy, gz);    // 200ms 기다리지 않고 즉시 전송
        // 낙상 후 움직임이 없는지 감시 시작 (의식불명 판정)
        // 보정 전에는 자이로 오프셋을 몰라 정지도 움직임으로 잡히므로 감시를 시작하지 않는다
        // (낙상 자체는 위에서 이미 즉시 전송됨)
        if (!unconscious && postureCalibrated) {
          unconsciousWatching = true;
          unconsciousWatchStartMs = now;
        }
      }
    }
    prevFallDetected = fall.fallDetected;
  }

  updateUnconsciousWatch(ax, ay, az, gx, gy, gz, now);

  if (bufCount < POSTURE_WINDOW_SIZE) return;  // 윈도우 아직 안 참

  PostureStatus posture = classifyPosture();
  bool healthAbnormal = checkHealthAbnormal();

  latestPosture = posture;
  latestHealthAbnormal = healthAbnormal;

  updateRestTracking(now);
  // 설계 원칙 1: 이상 감지되면 네트워크 상관없이 즉시 음성 안내
  // (음성은 별도 태스크에서 재생하고 같은 문장은 30초 안에 반복하지 않으므로 루프를 막지 않음)
  announceSituation(posture, healthAbnormal, now);
}

// ═══════════════════════════════════════════════════════
// 비콘 구역 판정 (core 0에서 별도 태스크로 계속 스캔)
// ═══════════════════════════════════════════════════════
/**
 * BLE 광고의 제조사 데이터가 우리 비콘(iBeacon) 프레임인지 검증하고 major를 파싱한다.
 * @param mfg 제조사 데이터 raw 바이트.
 * @param len mfg의 길이(바이트).
 * @param major [out] 파싱된 major 값 (유효할 때만 채워짐).
 * @return type/length/UUID가 전부 일치하면 true.
 */
bool parseIBeacon(const uint8_t* mfg, size_t len, uint16_t &major) {
  if (len < 25) return false;
  if (mfg[0] != TARGET_COMPANY_ID[0] || mfg[1] != TARGET_COMPANY_ID[1]) return false;
  if (mfg[2] != 0x02 || mfg[3] != 0x15) return false;
  for (int i = 0; i < 16; i++) {
    if (mfg[4 + i] != TARGET_UUID[i]) return false;
  }
  major = (mfg[20] << 8) | mfg[21];
  return true;
}

/** BLE 스캔 중 광고가 잡힐 때마다 호출되는 콜백. 구역 후보 갱신을 담당한다. */
class BeaconScanCallback : public BLEAdvertisedDeviceCallbacks {
  /**
   * 잡힌 광고가 우리 비콘이고 매핑된(known) major면, 지금까지 이번 라운드에서
   * 본 것 중 RSSI가 가장 세면 currentZoneMajor/currentZoneValid를 갱신한다.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (!advertisedDevice.haveManufacturerData()) return;
    String md = advertisedDevice.getManufacturerData();
    if (md.length() < 25) return;

    uint16_t major;
    if (!parseIBeacon((const uint8_t*)md.c_str(), md.length(), major)) return;
    // 매핑 안 된 major(새 비콘이 추가됐는데 코드가 아직 안 따라온 경우 등)는
    // 후보에서 제외 — 신호가 세더라도 이름 모르는 구역으로 갈아타지 않는다.
    if (strcmp(zoneNameForMajor(major), "UNKNOWN") == 0) return;

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

/**
 * core 0에서 도는 FreeRTOS 태스크. 3초씩 BLE 스캔을 반복하면서 currentZoneMajor를
 * 갱신하고, 오래(BEACON_STALE_MS) 갱신이 없으면 "구역 없음"으로 되돌린다.
 * 구역이 실제로 바뀌거나 만료될 때만 시리얼 로그를 남긴다.
 * @param param FreeRTOS 태스크 파라미터 (사용 안 함).
 */
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
    uint16_t zoneMajorCopy = currentZoneMajor;
    bool zoneValidCopy = currentZoneValid;
    xSemaphoreGive(zoneMutex);

    // 구역이 바뀌었거나(다른 major) 유효/만료 상태가 바뀌었을 때만 로그.
    if (zoneValidCopy != loggedZoneValid || zoneMajorCopy != loggedZoneMajor) {
      if (zoneValidCopy) {
        Serial.printf("[구역] %s (major=%u)\n", zoneNameForMajor(zoneMajorCopy), zoneMajorCopy);
      } else {
        Serial.println("[구역] 없음 (비콘 범위 밖)");
      }
      loggedZoneMajor = zoneMajorCopy;
      loggedZoneValid = zoneValidCopy;
    }

    vTaskDelay(pdMS_TO_TICKS(300));
  }
}
