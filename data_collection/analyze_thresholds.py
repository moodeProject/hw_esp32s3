"""
휘청거림/자세붕괴 임계값 튜닝용 분석 스크립트
────────────────────────────────────────────────
collect_session.py로 모은 라벨링된 CSV(normal / stagger / gradual_collapse / transition)를
posture_alert_logic.py의 classify_posture()와 동일한 피처(tilt_max, tilt_mean, gyro_std, slope)로
윈도우 단위 계산한 뒤, 라벨별 분포를 비교해서 THRESH_* 값을 얼마로 잡아야 할지 보여준다.

사용법:
    python analyze_thresholds.py                 # sessions/ 안의 모든 csv 합쳐서 분석
    python analyze_thresholds.py --file sessions/session_1_xxx.csv   # 특정 파일만

읽는 법:
  라벨별 tilt_max / gyro_std / tilt_mean / slope 표가 출력됨.
  - STUMBLE 기준(THRESH_STUMBLE_TILT_DEG, THRESH_STUMBLE_GYRO_STD)은
    normal 그룹과 stagger 그룹의 tilt_max / gyro_std 분포가 갈리는 지점으로 잡는다.
  - COLLAPSE 기준(THRESH_COLLAPSE_TILT_DEG, THRESH_COLLAPSE_SLOPE)은
    normal 그룹과 gradual_collapse 그룹의 tilt_mean / slope 분포가 갈리는 지점으로 잡는다.
  - "제안 임계값" 줄은 (normal 95퍼센타일 + 이상행동 5퍼센타일)의 중간값으로 자동 계산한 참고값.
    그대로 쓰지 말고 오탐/미탐 보면서 미세조정할 것.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8")

WINDOW_SIZE = 150     # posture_alert_logic.py의 POSTURE_WINDOW_SIZE와 동일 (3초 @ 50Hz)
STRIDE = 25            # 윈도우를 얼마나 겹치게 슬라이딩할지 (0.5초씩 이동)

SESSIONS_DIR = Path(__file__).parent / "sessions"


def tilt_angle_deg(ax, ay, az):
    acc_svm = np.sqrt(ax**2 + ay**2 + az**2) + 1e-6
    cos_tilt = np.clip(az / acc_svm, -1.0, 1.0)
    return np.degrees(np.arccos(cos_tilt))


def window_features(seg: pd.DataFrame) -> dict | None:
    """seg: 길이 WINDOW_SIZE인 연속 구간 (같은 라벨)."""
    if len(seg) < WINDOW_SIZE:
        return None

    ax, ay, az = seg["ax"].values, seg["ay"].values, seg["az"].values
    gx, gy, gz = seg["gx"].values, seg["gy"].values, seg["gz"].values

    tilt = tilt_angle_deg(ax, ay, az)
    gyro_svm = np.sqrt(gx**2 + gy**2 + gz**2)

    t = np.arange(len(tilt))
    slope = np.polyfit(t, tilt, 1)[0]

    return {
        "tilt_max": tilt.max(),
        "tilt_mean": tilt.mean(),
        "gyro_std": gyro_svm.std(),
        "slope": slope,
    }


def load_data(file_arg: str | None) -> pd.DataFrame:
    if file_arg:
        return pd.read_csv(file_arg)

    files = sorted(SESSIONS_DIR.glob("session_*.csv"))
    if not files:
        raise SystemExit(f"{SESSIONS_DIR} 안에 session_*.csv가 없습니다. 먼저 collect_session.py로 수집하세요.")

    dfs = [pd.read_csv(f) for f in files]
    print(f"{len(files)}개 세션 파일 로드: {[f.name for f in files]}")
    return pd.concat(dfs, ignore_index=True)


def extract_windows_per_label(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    # sessionId + 라벨이 바뀌는 지점마다 구간을 끊어서, 그 안에서만 슬라이딩 윈도우
    df = df.sort_values(["sessionId", "t_ms"]).reset_index(drop=True)
    df["_group"] = (df["label"] != df["label"].shift()) | (df["sessionId"] != df["sessionId"].shift())
    df["_segment_id"] = df["_group"].cumsum()

    for (label, seg_id), seg in df.groupby(["label", "_segment_id"]):
        if label == "transition":
            continue
        seg = seg.reset_index(drop=True)
        for start in range(0, len(seg) - WINDOW_SIZE + 1, STRIDE):
            window = seg.iloc[start:start + WINDOW_SIZE]
            feats = window_features(window)
            if feats:
                feats["label"] = label
                rows.append(feats)

    return pd.DataFrame(rows)


def suggest_threshold(normal_vals, abnormal_vals, direction="above"):
    """direction='above': abnormal이 더 크다고 가정 (tilt_max, gyro_std, tilt_mean, slope 전부 해당)."""
    normal_p95 = np.percentile(normal_vals, 95)
    abnormal_p5 = np.percentile(abnormal_vals, 5)
    if direction == "above" and abnormal_p5 > normal_p95:
        return (normal_p95 + abnormal_p5) / 2, True
    # 겹치는 경우 (분리가 깨끗하지 않음)
    return (normal_p95 + abnormal_p5) / 2, False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", default=None, help="특정 세션 csv 하나만 분석하고 싶을 때")
    args = parser.parse_args()

    df = load_data(args.file)
    windows = extract_windows_per_label(df)

    if windows.empty:
        raise SystemExit("윈도우를 하나도 못 만들었습니다. 세션 길이가 3초(150샘플)보다 짧은 구간만 있는지 확인하세요.")

    print("\n=== 라벨별 피처 분포 ===")
    summary = windows.groupby("label")[["tilt_max", "tilt_mean", "gyro_std", "slope"]].describe(
        percentiles=[0.05, 0.5, 0.95]
    )
    pd.set_option("display.width", 160)
    print(summary)

    print("\n=== 임계값 제안 (참고용, 오탐/미탐 보면서 재조정 필수) ===")

    if "normal" in windows["label"].unique():
        normal = windows[windows["label"] == "normal"]

        if "stagger" in windows["label"].unique():
            stagger = windows[windows["label"] == "stagger"]
            tilt_th, clean = suggest_threshold(normal["tilt_max"], stagger["tilt_max"])
            gyro_th, clean2 = suggest_threshold(normal["gyro_std"], stagger["gyro_std"])
            print(f"THRESH_STUMBLE_TILT_DEG 제안: {tilt_th:.1f}  (분리 {'깨끗함' if clean else '겹침 있음 — 재수집 고려'})")
            print(f"THRESH_STUMBLE_GYRO_STD 제안: {gyro_th:.1f}  (분리 {'깨끗함' if clean2 else '겹침 있음 — 재수집 고려'})")
        else:
            print("stagger(휘청거림) 라벨 데이터가 없습니다.")

        if "gradual_collapse" in windows["label"].unique():
            collapse = windows[windows["label"] == "gradual_collapse"]
            tilt_mean_th, clean3 = suggest_threshold(normal["tilt_mean"], collapse["tilt_mean"])
            slope_th, clean4 = suggest_threshold(normal["slope"], collapse["slope"])
            print(f"THRESH_COLLAPSE_TILT_DEG 제안: {tilt_mean_th:.1f}  (분리 {'깨끗함' if clean3 else '겹침 있음 — 재수집 고려'})")
            print(f"THRESH_COLLAPSE_SLOPE 제안: {slope_th:.3f}  (분리 {'깨끗함' if clean4 else '겹침 있음 — 재수집 고려'})")
        else:
            print("gradual_collapse(자세붕괴) 라벨 데이터가 없습니다.")
    else:
        print("normal 라벨 데이터가 없어서 비교 기준을 못 잡습니다.")


if __name__ == "__main__":
    main()
