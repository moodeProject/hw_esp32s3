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
]

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
