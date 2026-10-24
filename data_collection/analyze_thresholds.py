"""
휘청거림/자세붕괴 임계값 튜닝용 분석 스크립트
────────────────────────────────────────────────
collect_session.py로 모은 라벨링된 CSV(normal / stagger / gradual_collapse / transition)를
posture_alert_logic.py의 classify_posture()와 동일한 피처(tilt_max, tilt_mean, gyro_std, slope)로
윈도우 단위 계산한 뒤, 라벨별 분포를 비교해서 THRESH_* 값을 얼마로 잡아야 할지 보여준다.

사용법:
    python analyze_thresholds.py                 # sessions/ 안의 모든 csv 합쳐서 분석
    python analyze_thresholds.py --file sessions/session_1_xxx.csv   # 특정 파일만
    python analyze_thresholds.py --absolute      # 예전 방식(Z축 수직 가정 절대 기울기)

기울기 계산 방식:
  --relative(기본): 세션마다 calib_stand 구간 가속도 평균 방향을 착용 기준 자세(g0)로 잡고,
                    tilt = 현재 가속도 벡터와 g0 사이 각도. 펌웨어(postureAlert.ino)와 동일.
                    calib_stand 구간이 없는 옛 세션은 normal 라벨 중 가장 조용한 3초 윈도우의
                    평균으로 g0를 추정한다(경고 출력).
  --absolute      : tilt = acos(az/|a|). 센서가 비스듬히 붙어 있으면 정자세도 ~33°로 나옴.

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


ABSOLUTE_G0 = np.array([0.0, 0.0, 1.0])  # Z축 수직 가정 (예전 방식)


def tilt_angle_deg(ax, ay, az, g0=ABSOLUTE_G0):
    """현재 가속도 벡터와 기준 단위벡터 g0 사이 각도(도). g0 기본값이면 예전 절대 기울기."""
    acc_svm = np.sqrt(ax**2 + ay**2 + az**2) + 1e-6
    cos_tilt = np.clip((ax * g0[0] + ay * g0[1] + az * g0[2]) / acc_svm, -1.0, 1.0)
    return np.degrees(np.arccos(cos_tilt))


def _unit_mean(seg: pd.DataFrame) -> np.ndarray:
    v = seg[["ax", "ay", "az"]].to_numpy(dtype=float).mean(axis=0)
    return v / (np.linalg.norm(v) + 1e-12)


def estimate_g0(session: pd.DataFrame) -> tuple[np.ndarray, str]:
    """세션 하나의 기준 자세 g0와 출처 설명을 반환한다."""
    calib = session[session["phase"] == "calib_stand"]
    if len(calib) > 0:
        return _unit_mean(calib), f"calib_stand {len(calib)}샘플"

    # 옛 세션: normal 구간 중 자이로가 가장 조용한 3초 윈도우를 정지 자세로 간주
    normal = session[session["label"] == "normal"].reset_index(drop=True)
    if len(normal) >= WINDOW_SIZE:
        gyro = np.sqrt(normal["gx"] ** 2 + normal["gy"] ** 2 + normal["gz"] ** 2)
        start = int(gyro.rolling(WINDOW_SIZE).mean().idxmin()) - WINDOW_SIZE + 1
        return _unit_mean(normal.iloc[start:start + WINDOW_SIZE]), "추정(가장 조용한 normal 3초)"

    return ABSOLUTE_G0, "추정 불가 — 절대 기울기로 대체"


def window_features(seg: pd.DataFrame, g0: np.ndarray = ABSOLUTE_G0) -> dict | None:
    """seg: 길이 WINDOW_SIZE인 연속 구간 (같은 라벨). g0: 기울기 기준 단위벡터."""
    if len(seg) < WINDOW_SIZE:
        return None

    ax, ay, az = seg["ax"].values, seg["ay"].values, seg["az"].values
    gx, gy, gz = seg["gx"].values, seg["gy"].values, seg["gz"].values

    tilt = tilt_angle_deg(ax, ay, az, g0)
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


def extract_windows_per_label(df: pd.DataFrame, relative: bool = True) -> pd.DataFrame:
    rows = []
    # sessionId + 라벨이 바뀌는 지점마다 구간을 끊어서, 그 안에서만 슬라이딩 윈도우
    df = df.sort_values(["sessionId", "t_ms"]).reset_index(drop=True)

    g0_by_session = {}
    if relative:
        print("\n=== 세션별 기준 자세 g0 (상대 기울기) ===")
        for sid, session in df.groupby("sessionId"):
            g0, source = estimate_g0(session)
            g0_by_session[sid] = g0
            abs_tilt = np.degrees(np.arccos(np.clip(g0[2], -1.0, 1.0)))
            warn = "" if source.startswith("calib_stand") else "  [경고]"
            print(f"  session {sid}: g0=({g0[0]:+.3f}, {g0[1]:+.3f}, {g0[2]:+.3f}) "
                  f"절대기울기 {abs_tilt:5.1f}°  ← {source}{warn}")
    df["_group"] = (df["label"] != df["label"].shift()) | (df["sessionId"] != df["sessionId"].shift())
    df["_segment_id"] = df["_group"].cumsum()

    for (label, seg_id), seg in df.groupby(["label", "_segment_id"]):
        if label == "transition":
            continue
        seg = seg.reset_index(drop=True)
        for start in range(0, len(seg) - WINDOW_SIZE + 1, STRIDE):
            window = seg.iloc[start:start + WINDOW_SIZE]
            g0 = g0_by_session.get(window["sessionId"].iloc[0], ABSOLUTE_G0)
            feats = window_features(window, g0)
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
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--relative", dest="relative", action="store_true", default=True,
                      help="calib_stand 기준 자세(g0) 대비 상대 기울기 (기본)")
    mode.add_argument("--absolute", dest="relative", action="store_false",
                      help="Z축 수직 가정 절대 기울기 (예전 방식)")
    args = parser.parse_args()

    df = load_data(args.file)
    print(f"기울기 방식: {'상대(g0 기준)' if args.relative else '절대(Z축 기준)'}")
    windows = extract_windows_per_label(df, relative=args.relative)

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
