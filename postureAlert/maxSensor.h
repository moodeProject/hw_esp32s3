//   - Red/IR 값을 링버퍼에 쌓기
//   - 윈도우가 찼을 때만 BPM/SpO2 계산 (posture 3초 윈도우와는 별도 주기)
//   - 손가락/피부 미접촉(IR 너무 낮음) 시 계산 스킵
//   - MPU6050 자이로 std가 높을 때(움직임 심할 때)는 motion artifact로
//     간주해서 이번 윈도우 결과 버림 (posture_alert.ino의 gyroStd를
//     updateVitalsBuffer() 호출 시 인자로 받아서 판단)
//   - 최종적으로 bool 하나(이상 여부)만 checkHealthAbnormal()에 반환
//
// 사용법 (posture_alert.ino 쪽 수정은 최소):
//   #include "maxSensor.h"       -> 상단에 추가
//   initMaxSensor();             -> setup() 안에 한 줄 추가
//   updateVitalsBuffer(gyroStd); -> loop()의 readMpu() 직후 한 줄 추가
//   checkHealthAbnormal() 내부에서 isVitalsAbnormal() 호출
// ─────────────────────────────────────────────────────────

#ifndef MAX_SENSOR_H
#define MAX_SENSOR_H

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// ── 센서 인스턴스 ──────────────────────────────────────────
MAX30105 particleSensor;

// ── 윈도우 설정 ────────────────────────────────────────────
#define VITALS_WINDOW_SIZE 250

// ── 정상 범위 (일반 성인 안정 시 기준, 추후 실측 데이터로 재조정) ──
const float BPM_MIN_NORMAL   = 40.0f;
const float BPM_MAX_NORMAL   = 140.0f;
const float SPO2_MIN_NORMAL  = 92.0f;

// ── 미착용/신호없음 판정 임계값 ────────────────────────────
const long IR_FINGER_THRESHOLD = 15000;  // 이 값보다 낮으면 피부 미접촉으로 간주

// ── motion artifact 배제 임계값 ────────────────────────────
// PPG는 쉽게 깨지므로 더 보수적으로 잡음. 실착용 테스트하면서 조정 필요
const float GYRO_STD_MOTION_LIMIT = 20.0f;

// ── 개인 기준선(베이스라인) 설정 ─────────────────────────────
// 착용 시작 후 이 기간 동안의 HR/HRV 평균을 "이 사람의 정상 상태"로 저장.
// HRV 측정 표준(1996 Task Force 권고안)은 5분 이상을 권장하며 정확도도 더 높음.
// 다만 시연 시간 관계상 짧게(2~3분) 잡아둔 값 — 실제 운영/제출용으로는
// 5분 이상으로 늘리는 것을 권장.
const unsigned long BASELINE_DURATION_MS = 30000UL;  // [테스트용 임시] 30초 - 원래 150000UL(2분30초)로 되돌릴 것
const int BASELINE_MIN_SAMPLES = 5;  // 이 개수 이상 모여야 기준선 확정

// ── HRV 개인 기준선 이탈 판정 ─────────────────────────────
// baselineHrvMean 대비 이 비율 이상 떨어지면 이상(스트레스/피로 의심)으로 판단
const float HRV_DEVIATION_FACTOR = 0.5f;

// ── 온열질환 위험 판정 (활동량 대비 심박 지속 상승) ─────────
const float HEAT_HR_MARGIN = 20.0f;              // 기준선보다 이 값 이상 높으면 "상승" 후보
const float HEAT_LOW_ACTIVITY_GYRO_STD = 15.0f;  // 이 값 이하면 "활동량 낮음"으로 간주
const unsigned long HEAT_SUSTAINED_MS = 600000UL;  // 10분 이상 지속되면 경고 (데모 시 단축 가능)

// ── 링버퍼 ─────────────────────────────────────────────────
long bufIr[VITALS_WINDOW_SIZE];
long bufRed[VITALS_WINDOW_SIZE];
int vitalsIndex = 0;
int vitalsCount = 0;

// ── 최근 계산 결과 캐시 ────────────────────────────────────
float lastBpm = 0;
unsigned long lastBpmMs = 0;   // lastBpm 을 마지막으로 갱신한 시각 (오래된 심박 판정 방지용)
float lastSpo2 = 0;
bool lastVitalsAbnormal = false;

// ── heartRate.h 피크 검출용 상태 ───────────────────────────
long lastIrForBeat = 0;
float beatsPerMinute = 0;
int rateBuf[4];
int rateBufIndex = 0;
int rateCount = 0;   // rateBuf에 실제로 채워진 개수 (최대 4) - 빈칸(0)까지 평균내던 문제 수정

// 박동 간격은 millis() 대신 센서 샘플 번호로 계산
// (한 루프에서 샘플 여러 개를 읽으면 millis()가 같게 찍히기 때문)
#define MAX_SAMPLE_PERIOD_MS 10      // 400sps ÷ 평균4 = 100sps → 샘플 1개 = 10ms
unsigned long maxSampleNo = 0;       // 센서 샘플 번호 (시간 대신 사용)
unsigned long lastBeatSampleNo = 0;  // 직전 박동이 잡힌 샘플 번호

// ── HRV(RMSSD) 계산용 ─────────────────────────────────────
#define IBI_BUF_SIZE 10
long ibiBuf[IBI_BUF_SIZE];  // 최근 비트 간격(ms)
int ibiCount = 0;
float lastHrv = 0;  // 최근 RMSSD(ms)

// ── 개인 기준선 상태 ───────────────────────────────────────
bool wearStarted = false;
unsigned long wearStartMs = 0;
bool baselineReady = false;
float baselineHrSum = 0, baselineHrvSum = 0;
int baselineSampleCount = 0;
float baselineHrMean = 0;
float baselineHrvMean = 0;

// ── 온열질환 지속시간 타이머 ───────────────────────────────
bool heatConditionActive = false;
unsigned long heatConditionStartMs = 0;
bool lastHeatRiskAbnormal = false;

// ── 서버 전송용: 최근 HRV 이탈(피로도) 판정 캐시 ─────────────
bool lastHrvAbnormal = false;

// ═══════════════════════════════════════════════════════
// 초기화
// ═══════════════════════════════════════════════════════
bool initMaxSensor() {
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("[MAX30102] 연결 실패");
    return false;
  }

  particleSensor.setup(0x1F, 4, 2, 400, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println("[MAX30102] 초기화 완료");
  return true;
}

// HRV 기록 함수 전방 선언 (updateBpmFromSample에서 먼저 호출되기 때문)
void recordIbi(long deltaMs);

// ═══════════════════════════════════════════════════════
// BPM 계산
// ═══════════════════════════════════════════════════════
void updateBpmFromSample(long irValue) {
  if (checkForBeat(irValue)) {
    long delta = (long)(maxSampleNo - lastBeatSampleNo) * MAX_SAMPLE_PERIOD_MS;
    lastBeatSampleNo = maxSampleNo;

    float bpm = 60000.0f / delta;

    // 놓친 박동 / 가짜 박동 걸러내기
    static long lastGoodIbi = 0;  // 직전에 받은 정상 범위 박동 간격(ms)
    bool inRange = (delta >= 300 && delta <= 1500);                     // 40~200bpm만 인정
    bool consistent = (lastGoodIbi == 0) ||
                      (labs(delta - lastGoodIbi) < lastGoodIbi * 0.3f);  // 직전 간격과 30% 이내
    if (!inRange) return;
    lastGoodIbi = delta;          // 튄 값이어도 기준은 갱신 (계속 막히지 않게)
    if (!consistent) return;      // 튄 값은 BPM/HRV에 안 넣음

    rateBuf[rateBufIndex] = (int)bpm;
    rateBufIndex = (rateBufIndex + 1) % 4;
    if (rateCount < 4) rateCount++;

    int sum = 0;
    for (int i = 0; i < rateCount; i++) sum += rateBuf[i];
    beatsPerMinute = (float)sum / rateCount;

    recordIbi(delta);  // HRV 계산용 비트 간격 기록
  }
}

// ═══════════════════════════════════════════════════════
// HRV용 비트 간격(IBI) 기록 — 배열이 차면 앞으로 한 칸씩 밀고 맨 뒤에 추가
// ═══════════════════════════════════════════════════════
void recordIbi(long deltaMs) {
  if (ibiCount < IBI_BUF_SIZE) {
    ibiBuf[ibiCount++] = deltaMs;
  } else {
    for (int i = 1; i < IBI_BUF_SIZE; i++) ibiBuf[i - 1] = ibiBuf[i];
    ibiBuf[IBI_BUF_SIZE - 1] = deltaMs;
  }
}

// ═══════════════════════════════════════════════════════
// HRV(RMSSD) 계산 — 연속된 비트 간격 차이의 제곱평균제곱근
// ═══════════════════════════════════════════════════════
float calcHrvRmssd() {
  if (ibiCount < 3) return -1;  // 표본 부족, 계산 불가
  double sumSq = 0;
  for (int i = 1; i < ibiCount; i++) {
    double diff = ibiBuf[i] - ibiBuf[i - 1];
    sumSq += diff * diff;
  }
  return sqrt(sumSq / (ibiCount - 1));
}

// ═══════════════════════════════════════════════════════
// SpO2 계산 (Red/IR AC-DC 비율 기반 근사식)
// ═══════════════════════════════════════════════════════
// 필요시 나중에 실측 데이터로 계수 보정
float calcSpo2FromWindow() {
  double irMean = 0, redMean = 0;
  for (int i = 0; i < VITALS_WINDOW_SIZE; i++) {
    irMean += bufIr[i];
    redMean += bufRed[i];
  }
  irMean /= VITALS_WINDOW_SIZE;
  redMean /= VITALS_WINDOW_SIZE;

  double irAcSq = 0, redAcSq = 0;
  for (int i = 0; i < VITALS_WINDOW_SIZE; i++) {
    double dIr = bufIr[i] - irMean;
    double dRed = bufRed[i] - redMean;
    irAcSq += dIr * dIr;
    redAcSq += dRed * dRed;
  }
  double irAcRms = sqrt(irAcSq / VITALS_WINDOW_SIZE);
  double redAcRms = sqrt(redAcSq / VITALS_WINDOW_SIZE);

  if (irMean < 1 || redMean < 1 || irAcRms < 1e-6) return -1;  // 계산 불가

  double R = (redAcRms / redMean) / (irAcRms / irMean);

  // 표준 근사식 (SparkFun/Maxim 앱노트 기준)
  float spo2 = 110.0f - 25.0f * (float)R;
  if (spo2 > 100) spo2 = 100;
  if (spo2 < 0) spo2 = 0;
  return spo2;
}

// ═══════════════════════════════════════════════════════
// 매 loop()마다 호출 — 샘플 추가 + 실시간 BPM 갱신
// ═══════════════════════════════════════════════════════
void updateVitalsBuffer(float gyroStd) {
  // 새 값이 있을 때만 읽음 (getIR/getRed처럼 최대 0.25초씩 기다리지 않음)
  // → MAX30102가 잠깐 멈춰도 추락 감지 루프(50Hz)는 느려지지 않음
  particleSensor.check();
  if (!particleSensor.available()) return;   // 새 값이 없으면 이번 루프는 건너뜀

  long irValue = 0, redValue = 0;
  while (particleSensor.available()) {        // 쌓인 값 중 가장 최신 것만 사용
    irValue = particleSensor.getFIFOIR();
    redValue = particleSensor.getFIFORed();
    particleSensor.nextSample();
    maxSampleNo++;
    // 손가락/피부 접촉 상태에서만, 샘플 하나도 안 버리고 박동 검출에 넣음
    if (irValue > IR_FINGER_THRESHOLD) updateBpmFromSample(irValue);
  }

  bufIr[vitalsIndex] = irValue;
  bufRed[vitalsIndex] = redValue;
  vitalsIndex = (vitalsIndex + 1) % VITALS_WINDOW_SIZE;
  if (vitalsCount < VITALS_WINDOW_SIZE) vitalsCount++;

  if (vitalsCount < VITALS_WINDOW_SIZE) return;  // 윈도우 아직 안 참

  // 윈도우가 찬 뒤에는 250개(약 5초)마다 한 번만 판정 (매 샘플마다 판정/출력하던 문제 수정)
  static int samplesSinceWindow = 0;
  if (++samplesSinceWindow < VITALS_WINDOW_SIZE) return;
  samplesSinceWindow = 0;

  // ── 윈도우 하나 다 찼을 때 최종 판정 ──
  double irMeanCheck = 0;
  for (int i = 0; i < VITALS_WINDOW_SIZE; i++) irMeanCheck += bufIr[i];
  irMeanCheck /= VITALS_WINDOW_SIZE;

  bool noFinger = (irMeanCheck < IR_FINGER_THRESHOLD);
  bool tooMuchMotion = (gyroStd > GYRO_STD_MOTION_LIMIT);

  if (noFinger || tooMuchMotion) {
    // 재측정 필요 상태 - 판정 보류, 직전 값 유지하되 abnormal로 새로 올리진 않음
    Serial.println(noFinger ? "[MAX30102] 미착용/신호없음 - 재측정 필요"
                             : "[MAX30102] 모션 아티팩트 - 재측정 필요");
    return;
  }

  lastBpm = beatsPerMinute;
  lastBpmMs = millis();
  lastSpo2 = calcSpo2FromWindow();
  lastHrv = calcHrvRmssd();

  // ── 착용 시작 시점 기록 (첫 유효 윈도우 기준) ──
  if (!wearStarted) {
    wearStarted = true;
    wearStartMs = millis();
  }

  // ── 개인 기준선 수집 (착용 초반 BASELINE_DURATION_MS 동안) ──
  if (!baselineReady) {
    if (millis() - wearStartMs < BASELINE_DURATION_MS) {
      if (lastHrv > 0) {
        baselineHrSum += lastBpm;
        baselineHrvSum += lastHrv;
        baselineSampleCount++;
      }
    } else if (baselineSampleCount >= BASELINE_MIN_SAMPLES) {
      baselineHrMean = baselineHrSum / baselineSampleCount;
      baselineHrvMean = baselineHrvSum / baselineSampleCount;
      baselineReady = true;
      Serial.printf("[기준선 확정] HR=%.1f HRV=%.1fms (표본 %d개)\n",
                    baselineHrMean, baselineHrvMean, baselineSampleCount);
    }
    // 표본이 부족하면 baselineReady는 false로 유지, 다음 윈도우에서 계속 수집 시도
  }

  bool bpmAbnormal = (lastBpm > 0) && (lastBpm < BPM_MIN_NORMAL || lastBpm > BPM_MAX_NORMAL);
  bool spo2Abnormal = (lastSpo2 > 0) && (lastSpo2 < SPO2_MIN_NORMAL);

  // ── HRV 개인 기준선 이탈 판정 (절대값이 아니라 이 사람 기준선 대비) ──
  bool hrvAbnormal = baselineReady && lastHrv > 0
                      && (lastHrv < baselineHrvMean * (1.0f - HRV_DEVIATION_FACTOR));
  lastHrvAbnormal = hrvAbnormal;

  // ── 온열질환 위험 판정: 활동량은 낮은데 심박만 기준선보다 계속 높은 상태 ──
  bool heatConditionNow = baselineReady
                           && (lastBpm > baselineHrMean + HEAT_HR_MARGIN)
                           && (gyroStd < HEAT_LOW_ACTIVITY_GYRO_STD);

  if (heatConditionNow) {
    if (!heatConditionActive) {
      heatConditionActive = true;
      heatConditionStartMs = millis();
    }
    lastHeatRiskAbnormal = (millis() - heatConditionStartMs >= HEAT_SUSTAINED_MS);
  } else {
    heatConditionActive = false;
    lastHeatRiskAbnormal = false;
  }

  lastVitalsAbnormal = bpmAbnormal || spo2Abnormal || hrvAbnormal || lastHeatRiskAbnormal;
}

// ═══════════════════════════════════════════════════════
// checkHealthAbnormal()에서 호출할 최종 진입점
// ═══════════════════════════════════════════════════════
bool isVitalsAbnormal() {
  return lastVitalsAbnormal;
}

// ═══════════════════════════════════════════════════════
// 서버 전송용 getter — 원인별 값을 그대로 노출 (postureAlert.ino에서 사용)
// ═══════════════════════════════════════════════════════
float getLastHrv() {
  return lastHrv;
}

bool isFatigueAbnormal() {
  return lastHrvAbnormal;
}

bool isHeatRiskAbnormal() {
  return lastHeatRiskAbnormal;
}

#endif