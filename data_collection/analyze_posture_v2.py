"""
상대 기울기(*_v2) 세션으로 자세 판정 규칙을 평가한다. 펌웨어 postureAlert.ino 의 classifyPosture 와 같은 전처리
(센서 끊김 샘플 제거, 3샘플 중간값, 세션별 calib_stand 기준 자세 g0)를 거쳐 3초 윈도우 특징값을 뽑고,
동작(phase)별로 각 규칙이 몇 % 걸리는지 출력한다.

사용법: python analyze_posture_v2.py            (sessions/*_v2_*.csv 전부)
"""
import sys, numpy as np, pandas as pd
from pathlib import Path
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE)); sys.stdout.reconfigure(encoding="utf-8")
from collect_session import SCENARIOS, build_phase_windows, label_for_time
S = HERE / "sessions"
files = sorted(S.glob("session_*_v2_*.csv"))
W, STRIDE = 150, 25
def med3(a):
    out = a.copy()
    for i in range(2, len(a)): out[i] = np.median(a[i-2:i+1], axis=0)
    return out
rows = []
for fn in files:
    scen = fn.stem.split("_", 2)[2].rsplit("_", 2)[0]  # session_<id>_<scenario>_<날짜>_<시각>
    d = pd.read_csv(fn)
    if d["t_ms"].iloc[0] > 1000:  # 보드가 이미 측정 중이라 시작 시각이 밀린 세션: 첫 샘플을 t=20ms 로 맞추고 라벨 재계산
        win, _ = build_phase_windows(SCENARIOS[scen])
        d["t_ms"] = d["t_ms"] - d["t_ms"].iloc[0] + 20
        lp = [label_for_time(t / 1000, win) for t in d["t_ms"]]
        d["label"] = [a for a, b in lp]; d["phase"] = [b for a, b in lp]
    acc = np.sqrt(d.ax**2 + d.ay**2 + d.az**2)
    d = d[acc > 0.05].reset_index(drop=True)  # 센서 끊김(-0.0005 반복) 제거
    X = med3(d[["ax","ay","az","gx","gy","gz"]].to_numpy(float))
    d[["ax","ay","az","gx","gy","gz"]] = X
    c = d[d.phase == "calib_stand"]; g0 = c[["ax","ay","az"]].mean().to_numpy(); g0 /= np.linalg.norm(g0)
    goff = c[["gx","gy","gz"]].mean().to_numpy()
    d["seg"] = ((d.phase != d.phase.shift())).cumsum()
    for _, seg in d.groupby("seg"):
        ph, lb = seg.phase.iloc[0], seg.label.iloc[0]
        if lb == "transition" or ph == "calib_stand": continue
        A = seg[["ax","ay","az"]].to_numpy(); G = seg[["gx","gy","gz"]].to_numpy()
        for s in range(0, len(seg) - W + 1, STRIDE):
            a, g = A[s:s+W], G[s:s+W]
            sv = np.linalg.norm(a, axis=1)
            tilt = np.degrees(np.arccos(np.clip(a @ g0 / sv, -1, 1)))
            gsvm = np.linalg.norm(g, axis=1)               # 펌웨어와 동일(오프셋 포함)
            grot = np.linalg.norm(g - goff, axis=1)        # 오프셋 뺀 실제 회전
            rows.append(dict(scen=scen, phase=ph, label=lb,
                tilt_max=tilt.max(), tilt_mean=tilt.mean(), tilt_last1s=tilt[-50:].mean(),
                slope=np.polyfit(np.arange(W), tilt, 1)[0], gyro_std=gsvm.std(), gyro_max=grot.max(), gyro_mean=grot.mean(),
                acc_std=sv.std(), acc_max=sv.max(), acc_min=sv.min(), acc_range=sv.max()-sv.min()))
F = pd.DataFrame(rows)
pd.set_option("display.width", 250); pd.set_option("display.max_columns", 30)
feats = ["tilt_max","tilt_mean","tilt_last1s","slope","gyro_std","gyro_max","gyro_mean","acc_std","acc_range"]
print("윈도우 수:", F.groupby("label").size().to_dict())
print("\n=== 동작별 중앙값 ===")
print(F.groupby(["label","phase"])[feats].median().round(2).to_string())
def rate(mask, name):
    out = F.assign(hit=mask).groupby(["label","phase"]).hit.mean().round(2)
    print(f"\n--- {name}: 동작별 판정 비율 ---"); print(out.to_string())
rate((F.tilt_mean>40)&(F.slope>0) | ((F.tilt_max>80)&(F.gyro_std>45)), "1차 규칙(쓰러짐 40° / 휘청 80°+자이로std 45)")
rate((F.acc_range>=1.2)&(F.tilt_max>=60), "휘청: 가속도 변동폭 ≥1.2g & 기울기 최대 ≥60°")
rate((F.tilt_mean>35)&(F.tilt_last1s>40)&(F.gyro_max<150), "쓰러짐: 평균 >35° & 최근 1초 >40° & 회전 최대 <150°/s")
