// ─────────────────────────────────────────────────────────
// fallDetector.h — 온보드 낙상 감지 (FastAPI app.py의 /predict 이식)
// ─────────────────────────────────────────────────────────
// app.py와 동일한 흐름:
//   1) 최근 50개 샘플(1초, 50Hz) 윈도우
//   2) extract_features()  -> 피처 15개
//   3) scaler.transform()  -> (값 - 평균) / 표준편차
//   4) model.predict_proba -> 트리 200개의 "낙상 확률" 평균 = confidence
//   5) model.predict       -> confidence > 0.5 이면 낙상
//
// 모델 값(트리, scaler)은 fallModel.h에 들어있고,
// 그 파일은 ai_repo/exportFallModel.py가 model.pkl로부터 자동 생성한다.
// ─────────────────────────────────────────────────────────
#pragma once
#include <math.h>
#include "fallModel.h"

#define FALL_WINDOW_SIZE 50   // app.py WINDOW_SIZE와 반드시 동일

// 결과 구조체
struct FallResult {
  bool fallDetected;   // 낙상 여부
  float confidence;    // 낙상 확률 (0.0 ~ 1.0)
};

// ── 2) 피처 15개 계산 (app.py extract_features와 동일) ─────────
// window[i][0..5] = ax, ay, az, gx, gy, gz  (m/s^2, deg/s)
static void extractFallFeatures(const float window[][6], double out[FALL_N_FEATURES]) {
  const int n = FALL_WINDOW_SIZE;
  double accSvm[FALL_WINDOW_SIZE], gyroSvm[FALL_WINDOW_SIZE];
  double gravityDev[FALL_WINDOW_SIZE], vRatio[FALL_WINDOW_SIZE];

  for (int i = 0; i < n; i++) {
    double ax = window[i][0], ay = window[i][1], az = window[i][2];
    double gx = window[i][3], gy = window[i][4], gz = window[i][5];
    accSvm[i]     = sqrt(ax * ax + ay * ay + az * az);
    gyroSvm[i]    = sqrt(gx * gx + gy * gy + gz * gz);
    gravityDev[i] = fabs(accSvm[i] - 9.8);
    double accH   = sqrt(ax * ax + ay * ay);
    vRatio[i]     = fabs(az) / (accH + 1e-6);
  }

  // 평균
  double accSum = 0, gyroSum = 0, gdSum = 0, vrSum = 0;
  double accMax = accSvm[0], accMin = accSvm[0];
  double gyroMax = gyroSvm[0], gyroMin = gyroSvm[0];
  double gdMax = gravityDev[0];
  int freefallCount = 0, impactCount = 0;

  for (int i = 0; i < n; i++) {
    accSum += accSvm[i];
    gyroSum += gyroSvm[i];
    gdSum += gravityDev[i];
    vrSum += vRatio[i];
    if (accSvm[i] > accMax) accMax = accSvm[i];
    if (accSvm[i] < accMin) accMin = accSvm[i];
    if (gyroSvm[i] > gyroMax) gyroMax = gyroSvm[i];
    if (gyroSvm[i] < gyroMin) gyroMin = gyroSvm[i];
    if (gravityDev[i] > gdMax) gdMax = gravityDev[i];
    if (accSvm[i] < 3.0) freefallCount++;
    if (accSvm[i] > 20.0) impactCount++;
  }
  double accMean = accSum / n, gyroMean = gyroSum / n, vrMean = vrSum / n;

  // 표준편차
  double accSq = 0, gyroSq = 0, vrSq = 0;
  for (int i = 0; i < n; i++) {
    accSq  += (accSvm[i] - accMean) * (accSvm[i] - accMean);
    gyroSq += (gyroSvm[i] - gyroMean) * (gyroSvm[i] - gyroMean);
    vrSq   += (vRatio[i] - vrMean) * (vRatio[i] - vrMean);
  }

  // 순서는 fallModel.h / app.py 와 동일해야 함
  out[0]  = accMean;                  // acc_svm_mean
  out[1]  = sqrt(accSq / n);          // acc_svm_std
  out[2]  = accMax;                   // acc_svm_max
  out[3]  = accMin;                   // acc_svm_min
  out[4]  = gyroMean;                 // gyro_svm_mean
  out[5]  = sqrt(gyroSq / n);         // gyro_svm_std
  out[6]  = gyroMax;                  // gyro_svm_max
  out[7]  = gdMax;                    // gravity_dev_max
  out[8]  = gdSum / n;                // gravity_dev_mean
  out[9]  = vrMean;                   // v_ratio_mean
  out[10] = sqrt(vrSq / n);           // v_ratio_std
  out[11] = accMax - accMin;          // acc_svm_range
  out[12] = gyroMax - gyroMin;        // gyro_svm_range
  out[13] = (double)freefallCount / n;  // freefall_ratio
  out[14] = (double)impactCount / n;    // impact_ratio
}

// ── 4) 트리 하나 따라 내려가서 리프의 "낙상 확률" 반환 ──────────
static float walkFallTree(int treeIdx, const float x[FALL_N_FEATURES]) {
  uint16_t node = FALL_TREE_ROOT[treeIdx];
  while (FALL_NODE_FEATURE[node] != FALL_LEAF) {
    if (x[FALL_NODE_FEATURE[node]] <= FALL_NODE_THRESHOLD[node]) {
      node = FALL_NODE_LEFT[node];
    } else {
      node = FALL_NODE_RIGHT[node];
    }
  }
  return FALL_NODE_PROB[node];
}

// ── 전체: 윈도우 50개 -> 낙상 여부 + confidence ────────────────
FallResult predictFall(const float window[][6]) {
  double feat[FALL_N_FEATURES];
  extractFallFeatures(window, feat);

  // 3) scaler.transform: (값 - 평균) / 표준편차
  float scaled[FALL_N_FEATURES];
  for (int i = 0; i < FALL_N_FEATURES; i++) {
    scaled[i] = (float)((feat[i] - FALL_SCALER_MEAN[i]) / FALL_SCALER_SCALE[i]);
  }

  // 4) 트리 200개 평균
  float probSum = 0;
  for (int t = 0; t < FALL_N_TREES; t++) {
    probSum += walkFallTree(t, scaled);
  }

  FallResult r;
  r.confidence = probSum / FALL_N_TREES;
  r.fallDetected = r.confidence > 0.5f;
  return r;
}
