// ─────────────────────────────────────────────────────────
// MAX98357A(I2S 앰프) 스피커 단독 테스트 — 안내 음성 5종 재생
// ─────────────────────────────────────────────────────────
// 시리얼 모니터(115200, 줄바꿈 아무거나)에서 명령 입력:
//   1  -> "비정상적인 징후가 감지되었습니다. 조심해주세요."   (자세 or 건강 이상)
//   2  -> "즉시 작업을 중단하고 휴식을 취하세요."             (자세 + 건강 이상)
//   3  -> "의식불명으로 판별되어 관리자에게 연락을 취하고 있습니다." (의식불명)
//   4  -> "열사병 위험이 있으니 휴식을 취해주세요."           (폭염 + 고심박 + 휴식 없음)
//   5  -> "괜찮으시면 고개를 움직여 주세요."                 (낙상 후 무움직임 — 본인 확인)
//   t  -> 1kHz 테스트 톤 1초 (배선 확인용)
//   v 60        -> 볼륨 0~100 (%)
//   p D1 D2 D3  -> I2S 핀 변경 (BCLK LRC DIN 순서, D번호 또는 GPIO 숫자)
//
// 음성 데이터는 voice_tools/make_voice_clips.py 로 생성한 voiceClips.h (16kHz/16bit/mono).
// ─────────────────────────────────────────────────────────

#include <ESP_I2S.h>
#include "voiceClips.h"

// ── 기본 핀 (XIAO ESP32-S3 실배선 기준, D0=버저 / D4,D5=I2C 와 겹치지 않음) ──
int pinBclk = 7;   // D8
int pinLrc  = 8;   // D9
int pinDin  = 9;   // D10

// 100%에서는 앰프가 클리핑되면서 소리가 심하게 깨졌음(실측) — 40% 근처가 무난
int volumePercent = 40;

I2SClass i2s;
bool i2sReady = false;

/**
 * XIAO ESP32-S3 의 "D번호" 표기를 GPIO 번호로 바꾼다. 숫자만 오면 GPIO 번호로 그대로 쓴다.
 * @return GPIO 번호, 해석 불가면 -1.
 */
int parsePin(const String& token) {
  static const int XIAO_D_TO_GPIO[] = {1, 2, 3, 4, 5, 6, 43, 44, 7, 8, 9};
  if (token.length() == 0) return -1;
  if (token[0] == 'D' || token[0] == 'd') {
    int d = token.substring(1).toInt();
    if (d < 0 || d > 10) return -1;
    return XIAO_D_TO_GPIO[d];
  }
  return token.toInt();
}

/** 현재 핀 설정으로 I2S 를 (재)초기화한다. */
bool startI2s() {
  if (i2sReady) {
    i2s.end();
    i2sReady = false;
  }
  i2s.setPins(pinBclk, pinLrc, pinDin);
  if (!i2s.begin(I2S_MODE_STD, VOICE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("[I2S] 초기화 실패");
    return false;
  }
  i2sReady = true;
  Serial.printf("[I2S] 시작: BCLK=GPIO%d LRC=GPIO%d DIN=GPIO%d, %dHz\n",
                pinBclk, pinLrc, pinDin, VOICE_SAMPLE_RATE);
  return true;
}

/** PCM 샘플을 볼륨 적용해서 I2S 로 흘려보낸다 (재생 끝날 때까지 블로킹). */
void playPcm(const int16_t* samples, size_t length) {
  if (!i2sReady) return;
  const size_t CHUNK = 256;
  int16_t buf[CHUNK];
  int32_t gain = volumePercent * 256 / 100;  // 8.8 고정소수점
  for (size_t pos = 0; pos < length; pos += CHUNK) {
    size_t n = min(CHUNK, length - pos);
    for (size_t i = 0; i < n; i++) {
      buf[i] = (int16_t)(((int32_t)samples[pos + i] * gain) >> 8);
    }
    i2s.write((const uint8_t*)buf, n * sizeof(int16_t));
  }
  // DMA 버퍼에 남은 마지막 소리가 다음 재생 전까지 반복되지 않도록 무음으로 밀어낸다.
  memset(buf, 0, sizeof(buf));
  for (int i = 0; i < 8; i++) i2s.write((const uint8_t*)buf, sizeof(buf));
}

void playVoice(VoiceClip clip) {
  if (!i2sReady) {
    Serial.println("[재생] 실패 — I2S 미초기화 (p 명령으로 핀 확인)");
    return;
  }
  const VoiceClipData& c = VOICE_CLIPS[(int)clip];
  Serial.printf("[재생] %s (%.1fs)\n", c.text, c.length / (float)VOICE_SAMPLE_RATE);
  playPcm(c.samples, c.length);
  Serial.println("[재생] 완료");
}

void playTestTone() {
  static int16_t tone1k[VOICE_SAMPLE_RATE];
  for (int i = 0; i < VOICE_SAMPLE_RATE; i++) {
    tone1k[i] = (int16_t)(sinf(2 * PI * 1000.0f * i / VOICE_SAMPLE_RATE) * 20000);
  }
  Serial.println("[재생] 1kHz 테스트 톤 1초");
  playPcm(tone1k, VOICE_SAMPLE_RATE);
}

void printHelp() {
  Serial.println();
  Serial.println("=== MAX98357A 스피커 테스트 ===");
  const int clipCount = sizeof(VOICE_CLIPS) / sizeof(VOICE_CLIPS[0]);
  for (int i = 0; i < clipCount; i++) Serial.printf("  %d : %s\n", i + 1, VOICE_CLIPS[i].text);
  Serial.println("  t : 1kHz 테스트 톤");
  Serial.printf("  v <0~100> : 볼륨 (현재 %d%%)\n", volumePercent);
  Serial.printf("  p <BCLK> <LRC> <DIN> : 핀 변경 (현재 GPIO%d GPIO%d GPIO%d, 예: p D1 D2 D3)\n",
                pinBclk, pinLrc, pinDin);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char c = line[0];

  if (c >= '1' && c <= '5' && line.length() == 1) {
    playVoice((VoiceClip)(c - '1'));
  } else if (c == 't' || c == 'T') {
    playTestTone();
  } else if (c == 'v' || c == 'V') {
    int v = line.substring(1).toInt();
    volumePercent = constrain(v, 0, 100);
    Serial.printf("[볼륨] %d%%\n", volumePercent);
  } else if (c == 'p' || c == 'P') {
    String rest = line.substring(1);
    rest.trim();
    int s1 = rest.indexOf(' ');
    int s2 = rest.indexOf(' ', s1 + 1);
    if (s1 < 0 || s2 < 0) {
      Serial.println("형식: p <BCLK> <LRC> <DIN>  (예: p D1 D2 D3)");
      return;
    }
    int b = parsePin(rest.substring(0, s1));
    int l = parsePin(rest.substring(s1 + 1, s2));
    int d = parsePin(rest.substring(s2 + 1));
    if (b < 0 || l < 0 || d < 0) {
      Serial.println("핀 해석 실패");
      return;
    }
    pinBclk = b; pinLrc = l; pinDin = d;
    startI2s();
  } else {
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // USB CDC 연결 대기
  startI2s();
  printHelp();
}

void loop() {
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }
}
