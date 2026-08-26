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
const long IR_FINGER_THRESHOLD = 50000;  // 이 값보다 낮으면 피부 미접촉으로 간주

// ── motion artifact 배제 임계값 ────────────────────────────
// PPG는 쉽게 깨지므로 더 보수적으로 잡음. 실착용 테스트하면서 조정 필요
const float GYRO_STD_MOTION_LIMIT = 20.0f;

// ── 링버퍼 ─────────────────────────────────────────────────
long bufIr[VITALS_WINDOW_SIZE];
long bufRed[VITALS_WINDOW_SIZE];
int vitalsIndex = 0;
int vitalsCount = 0;

// ── 최근 계산 결과 캐시 ────────────────────────────────────
float lastBpm = 0;
float lastSpo2 = 0;
bool lastVitalsAbnormal = false;

// ── heartRate.h 피크 검출용 상태 ───────────────────────────
long lastIrForBeat = 0;
float beatsPerMinute = 0;
int rateBuf[4];
int rateBufIndex = 0;
long lastBeatMs = 0;

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

// ═══════════════════════════════════════════════════════
// BPM 계산
// ═══════════════════════════════════════════════════════
void updateBpmFromSample(long irValue) {
  if (checkForBeat(irValue)) {
    long now = millis();
    long delta = now - lastBeatMs;
    lastBeatMs = now;

    float bpm = 60.0f / (delta / 1000.0f);
    if (bpm > 20 && bpm < 255) {
      rateBuf[rateBufIndex++] = (int)bpm;
      rateBufIndex %= 4;

      int sum = 0;
      for (int i = 0; i < 4; i++) sum += rateBuf[i];
      beatsPerMinute = sum / 4.0f;
    }
  }
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
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // [디버그] 원시값 확인용 - 하드웨어 테스트 끝나면 지우기
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs > 200) {
    Serial.printf("[MAX30102 원시값] IR:%ld Red:%ld\n", irValue, redValue);
    lastPrintMs = millis();
  }

  bufIr[vitalsIndex] = irValue;
  bufRed[vitalsIndex] = redValue;
  vitalsIndex = (vitalsIndex + 1) % VITALS_WINDOW_SIZE;
  if (vitalsCount < VITALS_WINDOW_SIZE) vitalsCount++;

  // 손가락/피부 접촉 상태에서만 BPM 실시간 갱신
  if (irValue > IR_FINGER_THRESHOLD) {
    updateBpmFromSample(irValue);
  }

  if (vitalsCount < VITALS_WINDOW_SIZE) return;  // 윈도우 아직 안 참

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
  lastSpo2 = calcSpo2FromWindow();

  bool bpmAbnormal = (lastBpm > 0) && (lastBpm < BPM_MIN_NORMAL || lastBpm > BPM_MAX_NORMAL);
  bool spo2Abnormal = (lastSpo2 > 0) && (lastSpo2 < SPO2_MIN_NORMAL);

  lastVitalsAbnormal = bpmAbnormal || spo2Abnormal;
}

// ═══════════════════════════════════════════════════════
// checkHealthAbnormal()에서 호출할 최종 진입점
// ═══════════════════════════════════════════════════════
bool isVitalsAbnormal() {
  return lastVitalsAbnormal;
}

#endif