"""
휘청거림/점진적 자세 무너짐 데이터 수집 스크립트.

collect_mode.ino를 올린 ESP32-S3에 시리얼로 연결해서, 미리 정해둔 시나리오
(SCENARIOS)의 구간 순서대로 비프음으로 안내하며 혼자서도 라벨링된 데이터를
수집할 수 있게 한다. 버튼이나 2인 1조 없이, "구간 시간표"를 그대로 라벨에
매핑하는 방식이다.

사용법:
    python collect_session.py --port COM5                    # 기본: normal 프리셋
    python collect_session.py --port COM5 --scenario stagger
    python collect_session.py --port COM5 --scenario collapse
    python collect_session.py --port COM5 --scenario mixed

--scenario normal 은 "정상" 동작만 다양하게(걷기/앉기·서기/숙이기/고개돌림/
쭈그려앉기/도구사용) 모으기 위한 프리셋이다. 학습용 label은 전부 "normal"로
저장되고, 세부 동작 이름은 phase 컬럼에 별도로 남는다(다양성 확인/디버깅용).

수집된 파일: sessions/session_<id>_<scenario>_<timestamp>.csv
컬럼: t_ms,ax,ay,az,gx,gy,gz,label,phase,sessionId
"""

import argparse
import csv
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial

try:
    import winsound

    def beep(freq=880, duration_ms=250):
        winsound.Beep(freq, duration_ms)
except ImportError:  # Windows가 아닌 환경 대비
    def beep(freq=880, duration_ms=250):
        print("\a", end="", flush=True)


# 시나리오 프리셋: (phase 이름, 학습용 label, 지속시간_초).
# phase는 세부 동작 기록용, label은 실제 학습에 쓰이는 값(normal/stagger/gradual_collapse).
# 자유롭게 순서/길이/반복을 조정해서 여러 세션을 돌리면 된다.
SCENARIOS = {
    # 정상 동작을 최대한 다양하게 모으는 프리셋 (여러 번 반복 실행 권장)
    "normal": [
        ("walk", "normal", 15),
        ("sit_stand", "normal", 12),
        ("bend_pickup", "normal", 12),
        ("head_turn_look_around", "normal", 10),
        ("kneel_squat", "normal", 12),
        ("tool_use_arm_motion", "normal", 12),
    ],
    "stagger": [
        ("normal", "normal", 5),
        ("stagger_forward", "stagger", 7),
        ("normal", "normal", 5),
        ("stagger_backward", "stagger", 7),
        ("normal", "normal", 5),
        ("stagger_lateral", "stagger", 7),
        ("normal", "normal", 5),
    ],
    "collapse": [
        ("normal", "normal", 5),
        ("slump_forward", "gradual_collapse", 6),
        ("normal", "normal", 5),
        ("slide_down_wall", "gradual_collapse", 6),
        ("normal", "normal", 5),
    ],
    # 세 클래스를 섞어서 빠르게 파일럿 점검할 때 쓰는 프리셋
    "mixed": [
        ("normal", "normal", 8),
        ("stagger", "stagger", 4),
        ("normal", "normal", 8),
        ("gradual_collapse", "gradual_collapse", 6),
        ("normal", "normal", 8),
    ],
}

# phase 이름 -> 사람이 읽고 바로 따라할 수 있는 동작 안내문
PHASE_INSTRUCTIONS = {
    "walk": "자유롭게 걸어다니세요",
    "sit_stand": "앉았다 일어나기를 반복하세요",
    "bend_pickup": "몸을 숙여 물건을 줍는 동작을 반복하세요",
    "head_turn_look_around": "고개를 좌우로 돌리며 주변을 둘러보세요",
    "kneel_squat": "쭈그려 앉거나 무릎을 꿇었다 일어나기를 반복하세요",
    "tool_use_arm_motion": "팔을 뻗어 도구를 사용하는 동작을 반복하세요",
    "normal": "평소처럼 자연스럽게 움직이세요 (걷기/작업 등)",
    "stagger_forward": "앞으로 휘청거리다가 균형을 잡으세요",
    "stagger_backward": "뒤로 휘청거리다가 균형을 잡으세요",
    "stagger_lateral": "옆으로 휘청거리다가 균형을 잡으세요",
    "slump_forward": "천천히 앞으로 무너지듯 주저앉으세요",
    "slide_down_wall": "벽을 짚고 천천히 미끄러지듯 주저앉으세요",
    "stagger": "휘청거리는 동작을 하세요",
    "gradual_collapse": "천천히 쓰러지듯 무너지는 동작을 하세요",
}

# 구간 경계 앞뒤로 이 시간만큼은 "transition"으로 표시해 학습 시 제외를 권장한다.
BOUNDARY_BUFFER_SEC = 1.0

SESSIONS_DIR = Path(__file__).parent / "sessions"


def build_phase_windows(scenario):
    """[(phase, label, start_sec, end_sec), ...] 형태로 누적 시간표를 만든다."""
    windows = []
    t = 0.0
    for phase, label, duration in scenario:
        windows.append((phase, label, t, t + duration))
        t += duration
    return windows, t


def label_for_time(t_sec, windows):
    """(label, phase) 튜플을 반환. 구간 경계 버퍼 구간은 label/phase 모두 'transition'."""
    for phase, label, start, end in windows:
        if start <= t_sec < end:
            if (t_sec - start) < BOUNDARY_BUFFER_SEC or (end - t_sec) < BOUNDARY_BUFFER_SEC:
                return "transition", "transition"
            return label, phase
    return "transition", "transition"


def reader_thread(ser, row_queue, stop_event):
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            continue
        if not line or line == "READY":
            continue
        parts = line.split(",")
        if len(parts) != 7:
            continue
        try:
            t_ms = int(parts[0])
            ax, ay, az, gx, gy, gz = (float(x) for x in parts[1:])
        except ValueError:
            continue
        row_queue.put((t_ms, ax, ay, az, gx, gy, gz))


def next_session_id():
    SESSIONS_DIR.mkdir(parents=True, exist_ok=True)
    counter_file = SESSIONS_DIR / ".next_id"
    current = int(counter_file.read_text()) if counter_file.exists() else 1
    counter_file.write_text(str(current + 1))
    return current


def run_session(port, baud, scenario_name):
    windows, total_duration = build_phase_windows(SCENARIOS[scenario_name])
    session_id = next_session_id()

    print(f"[세션 {session_id}] 시나리오: {scenario_name}")
    for phase, label, start, end in windows:
        print(f"  {start:5.1f}s ~ {end:5.1f}s : {phase} (label={label})")
    print(f"총 소요 시간: {total_duration:.1f}초\n")

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # 보드 리셋 대기

    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if line != "READY":
        print(f"[경고] 보드로부터 예상한 READY 대신 '{line}' 수신 — 계속 진행합니다.")

    input("헬멧을 착용하고 준비되면 Enter를 누르세요 (누르는 즉시 시나리오가 시작됩니다)...")

    row_queue = queue.Queue()
    stop_event = threading.Event()
    t_reader = threading.Thread(target=reader_thread, args=(ser, row_queue, stop_event), daemon=True)
    t_reader.start()

    ser.write(b"s")
    session_start = time.time()

    def wait_until(target_time):
        remaining = target_time - time.time()
        if remaining > 0:
            time.sleep(remaining)

    for phase, label, start, end in windows:
        wait_until(session_start + start)
        beep(880 if label == "normal" else 1400, 200)
        instruction = PHASE_INSTRUCTIONS.get(phase, phase)
        print(f"\n>>> [{phase}] {instruction}  (label={label})")

        phase_end = session_start + end
        while True:
            remaining = phase_end - time.time()
            if remaining <= 0:
                break
            print(f"\r    남은 시간: {remaining:4.1f}s   ", end="", flush=True)
            time.sleep(0.1)
        print("\r    구간 종료          ")

    beep(2000, 400)
    print("=== 시나리오 종료 ===")

    time.sleep(0.5)  # 마지막 샘플까지 큐에 들어올 시간 확보
    stop_event.set()
    ser.close()

    rows = []
    while not row_queue.empty():
        rows.append(row_queue.get())
    rows.sort(key=lambda r: r[0])

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = SESSIONS_DIR / f"session_{session_id}_{scenario_name}_{timestamp}.csv"
    with out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ms", "ax", "ay", "az", "gx", "gy", "gz", "label", "phase", "sessionId"])
        for t_ms, ax, ay, az, gx, gy, gz in rows:
            label, phase = label_for_time(t_ms / 1000.0, windows)
            writer.writerow([t_ms, ax, ay, az, gx, gy, gz, label, phase, session_id])

    print(f"저장 완료: {out_path} ({len(rows)} 샘플)")


def main():
    parser = argparse.ArgumentParser(description="헬멧 IMU 라벨링 데이터 수집")
    parser.add_argument("--port", required=True, help="ESP32-S3 시리얼 포트 (예: COM5)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--scenario",
        default="normal",
        choices=sorted(SCENARIOS.keys()),
        help="수집할 시나리오 프리셋 (기본: normal)",
    )
    args = parser.parse_args()

    try:
        run_session(args.port, args.baud, args.scenario)
    except KeyboardInterrupt:
        print("\n중단되었습니다.")
        sys.exit(1)


if __name__ == "__main__":
    main()
