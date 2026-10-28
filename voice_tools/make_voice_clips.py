"""
헬멧 안내 음성(한국어) 생성기.

Windows 내장 TTS(Microsoft Heami, ko-KR)로 문장을 WAV로 만든 뒤
16kHz / 16bit / mono PCM으로 변환해서 ESP32 스케치가 플래시에 그대로 넣을 수 있는
C 헤더(voiceClips.h)로 출력한다. 문장을 바꾸면 이 스크립트만 다시 돌리면 된다.

사용법:
    python make_voice_clips.py <출력 헤더 경로>

현재 출력 위치: postureAlert/voiceClips.h (실사용), speakerTest/voiceClips.h (단독 테스트)
"""
import os
import subprocess
import sys
import tempfile

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly

SAMPLE_RATE = 16000

# (C 식별자, 문장) — 순서가 곧 VoiceClip enum 순서
CLIPS = [
    ("WARN_ABNORMAL", "비정상적인 징후가 감지되었습니다. 조심해주세요."),
    ("STOP_AND_REST", "즉시 작업을 중단하고 휴식을 취하세요."),
    ("UNCONSCIOUS", "의식불명으로 판별되어 관리자에게 연락을 취하고 있습니다."),
    ("HEATSTROKE", "열사병 위험이 있으니 휴식을 취해주세요."),
    ("CHECK_RESPONSE", "괜찮으시면 고개를 움직여 주세요."),
    # 헬멧 카메라 영상에서 위험이 확정됐을 때(서버 응답 hazardWarning). 앞에 경고음 3번을 붙인다.
    ("HAZARD_AHEAD", "전방에 위험 요소가 있습니다. 조심하세요."),
    # 켜진 뒤 3초 정지로 기준 자세가 잡혔을 때. 이때부터 자세·낙상 판정이 동작한다.
    ("CALIB_DONE", "기준 자세가 저장되었습니다."),
]

# 문장 앞에 경고음(띠띠띠)을 붙일 클립
BEEP_PREFIX = {"HAZARD_AHEAD": 3}

PS_TEMPLATE = r"""
Add-Type -AssemblyName System.Speech
$s = New-Object System.Speech.Synthesis.SpeechSynthesizer
$s.SelectVoice('Microsoft Heami Desktop')
$s.Rate = 0
$fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
$s.SetOutputToWaveFile('{out}', $fmt)
$s.Speak([IO.File]::ReadAllText('{txt}', [Text.Encoding]::UTF8))
$s.Dispose()
"""


def synthesize(text: str, out_wav: str, workdir: str) -> None:
    txt_path = os.path.join(workdir, "line.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(text)
    script = PS_TEMPLATE.format(out=out_wav, txt=txt_path)
    subprocess.run(["powershell", "-NoProfile", "-Command", script], check=True)


def load_pcm16(path: str) -> np.ndarray:
    rate, data = wavfile.read(path)
    if data.ndim > 1:
        data = data.mean(axis=1)
    data = data.astype(np.float32)
    if rate != SAMPLE_RATE:
        data = resample_poly(data, SAMPLE_RATE, rate)
    # 앞뒤 무음 정리 + 피크 정규화(-1dBFS)
    nz = np.nonzero(np.abs(data) > 200)[0]
    if len(nz):
        data = data[max(0, nz[0] - 800): nz[-1] + 1600]
    peak = np.max(np.abs(data)) or 1.0
    data = data / peak * 32767 * 0.89
    return data.astype(np.int16)


def beeps(count: int) -> np.ndarray:
    """1.2kHz 짧은 경고음 count 번 + 문장 앞 쉼. 앞뒤를 살짝 줄여 딸깍 소리를 없앤다."""
    t = np.arange(int(SAMPLE_RATE * 0.12)) / SAMPLE_RATE
    tone = np.sin(2 * np.pi * 1200 * t)
    fade = np.minimum(1.0, np.minimum(t, t[::-1]) / 0.005)
    tone = (tone * fade * 32767 * 0.7).astype(np.int16)
    gap = np.zeros(int(SAMPLE_RATE * 0.09), dtype=np.int16)
    tail = np.zeros(int(SAMPLE_RATE * 0.25), dtype=np.int16)
    return np.concatenate([np.concatenate([tone, gap])] * count + [tail])


def main() -> None:
    out_header = sys.argv[1] if len(sys.argv) > 1 else "voiceClips.h"
    lines = [
        "// 자동 생성 파일 — 직접 수정하지 말 것 (voice_tools/make_voice_clips.py)",
        f"// {SAMPLE_RATE}Hz / 16bit / mono PCM",
        "#pragma once",
        "#include <stdint.h>",
        "#include <stddef.h>",
        "",
        f"#define VOICE_SAMPLE_RATE {SAMPLE_RATE}",
        "",
    ]
    with tempfile.TemporaryDirectory() as tmp:
        for name, text in CLIPS:
            wav = os.path.join(tmp, f"{name}.wav")
            synthesize(text, wav, tmp)
            pcm = load_pcm16(wav)
            if name in BEEP_PREFIX:
                pcm = np.concatenate([beeps(BEEP_PREFIX[name]), pcm])
            print(f"{name}: {len(pcm) / SAMPLE_RATE:.2f}s, {pcm.nbytes} bytes")
            lines.append(f"// \"{text}\"")
            lines.append(f"static const int16_t VOICE_{name}[{len(pcm)}] = {{")
            for i in range(0, len(pcm), 24):
                lines.append("  " + ",".join(str(v) for v in pcm[i:i + 24]) + ",")
            lines.append("};")
            lines.append("")

    lines.append("enum class VoiceClip { " + ", ".join(n for n, _ in CLIPS) + " };")
    lines.append("")
    lines.append("struct VoiceClipData { const int16_t* samples; size_t length; const char* text; };")
    lines.append("static const VoiceClipData VOICE_CLIPS[] = {")
    for name, text in CLIPS:
        lines.append(f"  {{ VOICE_{name}, sizeof(VOICE_{name}) / sizeof(int16_t), \"{text}\" }},")
    lines.append("};")
    lines.append("")

    with open(out_header, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"-> {out_header}")


if __name__ == "__main__":
    main()
