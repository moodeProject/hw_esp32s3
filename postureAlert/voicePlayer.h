// ─────────────────────────────────────────────────────────
// voicePlayer.h — MAX98357A(I2S 앰프) 안내 음성 재생
// ─────────────────────────────────────────────────────────
// 음성 한 문장이 3~5초라서 loop()에서 그대로 재생하면 50Hz IMU 샘플링/낙상 판정이
// 그동안 멈춘다. 그래서 재생은 별도 FreeRTOS 태스크가 맡고, loop()는 requestVoice()로
// "이 음성 틀어줘" 요청만 큐에 넣고 바로 돌아간다.
//
// 사용법:
//   initVoicePlayer();             -> setup()
//   requestVoice(VoiceClip::...);  -> 아무 데서나 (논블로킹)
//
// 음성 데이터(voiceClips.h)는 voice_tools/make_voice_clips.py 로 생성 (16kHz/16bit/mono).
// 배선/볼륨 값은 speakerTest 스케치로 실측 확인한 값.
// ─────────────────────────────────────────────────────────
#pragma once
#include <ESP_I2S.h>
#include "voiceClips.h"

// ── 핀 (XIAO ESP32-S3) ─────────────────────────────────────
#define VOICE_PIN_BCLK 7   // D8
#define VOICE_PIN_LRC  8   // D9
#define VOICE_PIN_DIN  9   // D10

// 100%에서는 앰프가 클리핑되면서 소리가 심하게 깨졌음(실측) — 40% 근처가 무난
#define VOICE_VOLUME_PERCENT 40

I2SClass voiceI2s;
QueueHandle_t voiceQueue = NULL;
volatile bool voicePlaying = false;

/** PCM 샘플을 볼륨 적용해서 I2S 로 흘려보낸다 (재생 끝날 때까지 블로킹 — 태스크 안에서만 호출). */
static void voicePlayPcm(const int16_t* samples, size_t length) {
  const size_t CHUNK = 256;
  int16_t buf[CHUNK];
  const int32_t gain = VOICE_VOLUME_PERCENT * 256 / 100;  // 8.8 고정소수점
  for (size_t pos = 0; pos < length; pos += CHUNK) {
    size_t n = min(CHUNK, length - pos);
    for (size_t i = 0; i < n; i++) {
      buf[i] = (int16_t)(((int32_t)samples[pos + i] * gain) >> 8);
    }
    voiceI2s.write((const uint8_t*)buf, n * sizeof(int16_t));
  }
  // DMA 버퍼에 남은 마지막 소리가 반복되지 않도록 무음으로 밀어낸다.
  memset(buf, 0, sizeof(buf));
  for (int i = 0; i < 8; i++) voiceI2s.write((const uint8_t*)buf, sizeof(buf));
}

static void voiceTask(void* param) {
  int clipIdx;
  for (;;) {
    if (xQueueReceive(voiceQueue, &clipIdx, portMAX_DELAY) != pdTRUE) continue;
    const VoiceClipData& c = VOICE_CLIPS[clipIdx];
    voicePlaying = true;
    Serial.printf("[음성] %s\n", c.text);
    voicePlayPcm(c.samples, c.length);
    voicePlaying = false;
  }
}

/**
 * I2S 와 재생 태스크를 띄운다. 실패해도 나머지 기능(센서/전송)은 계속 동작해야 하므로
 * false 만 돌려주고 멈추지 않는다.
 */
bool initVoicePlayer() {
  voiceI2s.setPins(VOICE_PIN_BCLK, VOICE_PIN_LRC, VOICE_PIN_DIN);
  if (!voiceI2s.begin(I2S_MODE_STD, VOICE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("[음성] I2S 초기화 실패 — 음성 안내 비활성화");
    return false;
  }
  // 큐 길이 1: 재생 중에 여러 요청이 쌓이면 가장 최근 것 하나만 남긴다 (xQueueOverwrite).
  voiceQueue = xQueueCreate(1, sizeof(int));
  // 우선순위 3: 같은 core 0의 비콘 스캔/기상 조회 태스크(우선순위 1)에 밀려 I2S 버퍼가
  // 비면 재생 중 지직거릴 수 있어서 그보다 높게 둔다. 재생 중엔 I2S write에서
  // 블로킹되므로 다른 태스크를 굶기지 않는다.
  if (voiceQueue == NULL ||
      xTaskCreatePinnedToCore(voiceTask, "VoicePlayer", 4096, NULL, 3, NULL, 0) != pdPASS) {
    Serial.println("[음성] 재생 태스크 생성 실패 — 음성 안내 비활성화");
    voiceQueue = NULL;
    return false;
  }
  Serial.println("[음성] MAX98357A 초기화 완료 (core 0)");
  return true;
}

/** 음성 재생 요청 (논블로킹). 재생 중이면 지금 문장이 끝난 뒤 재생된다. */
void requestVoice(VoiceClip clip) {
  if (voiceQueue == NULL) return;
  int idx = (int)clip;
  xQueueOverwrite(voiceQueue, &idx);
}
