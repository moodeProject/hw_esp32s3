#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define MPU_ADDR 0x68
#define SDA_PIN 5
#define SCL_PIN 6

const char* ssid      = //TODO:이름 입력 ;
const char* password  = //TODO:비밀번호 입력 "";
const char* serverURL = //TODO: IP주소 입력 "http://[]:8080/api/sensor-data";
const char* deviceId  = "HELMET-001";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== 부팅 시작 ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  Serial.println("MPU6050 초기화 완료");

  // Wi-Fi 타임아웃 추가 (20초)
  WiFi.begin(ssid, password);
  Serial.print("Wi-Fi 연결 중");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  Serial.println();

Serial.print("WiFi.status() = ");



if (WiFi.status() == WL_CONNECTED) {

  Serial.println("Wi-Fi 연결 완료!");

} else {

  Serial.println("Wi-Fi 실패");

}
}

void loop() {
  Serial.println("--- loop 시작 ---");  // 여기 뜨는지 먼저 확인

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

  float ax = ax_raw / 16384.0;
  float ay = ay_raw / 16384.0;
  float az = az_raw / 16384.0;
  float gx = gx_raw / 131.0;
  float gy = gy_raw / 131.0;
  float gz = gz_raw / 131.0;

  Serial.printf("[센서] ax:%.2f ay:%.2f az:%.2f | gx:%.2f gy:%.2f gz:%.2f\n",
                ax, ay, az, gx, gy, gz);

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<200> doc;
    doc["deviceId"] = deviceId;
    doc["ax"] = ax;
    doc["ay"] = ay;
    doc["az"] = az;
    doc["gx"] = gx;
    doc["gy"] = gy;
    doc["gz"] = gz;

    String body;
    serializeJson(doc, body);
    Serial.println("[전송] " + body);

    int responseCode = http.POST(body);
    Serial.printf("[응답코드] %d\n", responseCode);

    if (responseCode > 0) {
      Serial.println("[응답] " + http.getString());
    } else {
      Serial.println("[오류] 서버 연결 실패");
    }
    http.end();
  } else {
    Serial.println(WiFi.status());  // setup() 끝부분에 추가
    Serial.println("[Wi-Fi 없음] 전송 스킵");
  }

  delay(1000);
}