// ─────────────────────────────────────────────────────────
// cameraStream.h — 헬멧 카메라(XIAO ESP32-S3 Sense) MJPEG 스트림
// ─────────────────────────────────────────────────────────
// 라즈베리파이 relay(ai_repo/pi/run.sh)가 같은 Wi-Fi에서
//   http://<ip>/status 응답에 "framesize" 가 있는 기기를 카메라로 찾고
//   http://<ip>:81/stream 을 읽어 AI 서버로 넘긴다.
// 전시 때 쓰던 CameraWebServer 예제와 같은 주소 규칙을 그대로 맞췄다. 그래서 센서 펌웨어와
// 카메라를 한 보드에서 같이 돌려도 Pi 쪽은 손대지 않아도 된다.
//
// 사용법: Wi-Fi 연결 뒤 startCameraStream() 한 번.
// 빌드 옵션에 PSRAM 필수 (FQBN ...,PSRAM=opi). 프레임 버퍼를 PSRAM 에 둔다.
// ─────────────────────────────────────────────────────────
#pragma once
#include "esp_camera.h"
#include "esp_http_server.h"

// XIAO ESP32-S3 Sense 카메라 핀 (CameraWebServer 예제 camera_pins.h 와 동일)
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  10
#define CAM_PIN_SIOD  40
#define CAM_PIN_SIOC  39
#define CAM_PIN_Y9    48
#define CAM_PIN_Y8    11
#define CAM_PIN_Y7    12
#define CAM_PIN_Y6    14
#define CAM_PIN_Y5    16
#define CAM_PIN_Y4    18
#define CAM_PIN_Y3    17
#define CAM_PIN_Y2    15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF  47
#define CAM_PIN_PCLK  13

// 데모 런북 값: VGA(640x480), 화질 12. 1080p 를 그대로 보내면 AI 서버 쪽이 6fps 로 느려졌다.
#define CAM_FRAME_SIZE   FRAMESIZE_VGA
#define CAM_JPEG_QUALITY 12

static bool cameraReady = false;

static esp_err_t camStatusHandler(httpd_req_t* req) {
  char json[96];
  snprintf(json, sizeof(json), "{\"framesize\":%d,\"quality\":%d,\"ready\":%s}",
           (int)CAM_FRAME_SIZE, CAM_JPEG_QUALITY, cameraReady ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

#define CAM_PART_BOUNDARY "123456789000000000000987654321"
static const char* CAM_STREAM_TYPE = "multipart/x-mixed-replace;boundary=" CAM_PART_BOUNDARY;
static const char* CAM_STREAM_SEP  = "\r\n--" CAM_PART_BOUNDARY "\r\n";
static const char* CAM_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/** 클라이언트(Pi)가 끊을 때까지 프레임을 계속 보낸다. httpd 자체 태스크에서 돈다. */
static esp_err_t camStreamHandler(httpd_req_t* req) {
  if (!cameraReady) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera not ready");
    return ESP_FAIL;
  }
  esp_err_t res = httpd_resp_set_type(req, CAM_STREAM_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char part[64];

  while (res == ESP_OK) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[카메라] 프레임 획득 실패");
      res = ESP_FAIL;
      break;
    }
    size_t hlen = snprintf(part, sizeof(part), CAM_STREAM_PART, (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, CAM_STREAM_SEP, strlen(CAM_STREAM_SEP));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }
  return res;
}

/**
 * 카메라와 스트림 서버(:80 /status, :81 /stream)를 띄운다.
 * 실패해도 센서/음성/전송은 그대로 동작해야 하므로 false 만 돌려주고 멈추지 않는다.
 */
bool startCameraStream() {
  if (!psramFound()) {
    Serial.println("[카메라] PSRAM 없음 — 빌드 옵션 PSRAM=opi 확인. 카메라 비활성화");
    return false;
  }

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_Y2;  config.pin_d1 = CAM_PIN_Y3;
  config.pin_d2 = CAM_PIN_Y4;  config.pin_d3 = CAM_PIN_Y5;
  config.pin_d4 = CAM_PIN_Y6;  config.pin_d5 = CAM_PIN_Y7;
  config.pin_d6 = CAM_PIN_Y8;  config.pin_d7 = CAM_PIN_Y9;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.pin_sccb_sda = CAM_PIN_SIOD;
  config.pin_sccb_scl = CAM_PIN_SIOC;
  config.sccb_i2c_port = 1;  // I2C0 은 MPU6050/MAX30102(Wire, D4/D5) 가 쓰고 있다
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = CAM_FRAME_SIZE;
  config.jpeg_quality = CAM_JPEG_QUALITY;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;  // 밀린 프레임 대신 항상 지금 장면

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[카메라] 초기화 실패 0x%x — 카메라 모듈 연결 확인\n", err);
    return false;
  }

  // 이 헬멧 모듈은 OV3660. 예제와 같이 상하 반전·밝기·채도를 보정한다.
  sensor_t* s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  cameraReady = true;

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;
  httpd_handle_t statusHttpd = NULL;
  httpd_uri_t statusUri = { "/status", HTTP_GET, camStatusHandler, NULL };
  if (httpd_start(&statusHttpd, &cfg) == ESP_OK) {
    httpd_register_uri_handler(statusHttpd, &statusUri);
  }

  cfg.server_port = 81;
  cfg.ctrl_port = 32769;
  httpd_handle_t streamHttpd = NULL;
  httpd_uri_t streamUri = { "/stream", HTTP_GET, camStreamHandler, NULL };
  if (httpd_start(&streamHttpd, &cfg) == ESP_OK) {
    httpd_register_uri_handler(streamHttpd, &streamUri);
  }

  Serial.printf("[카메라] 스트림 시작: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  return true;
}
