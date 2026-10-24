// 데이터 수집 전용 펌웨어 (WiFi 없음, Serial로 CSV 스트리밍)
// 호스트(collect_session.py)가 's' 1바이트를 보내면 그 순간을 t=0으로 잡고
// 이후 50Hz(20ms)로 MPU6050 원시값을 CSV 한 줄씩 출력한다.
// 라인 포맷: t_ms,ax,ay,az,gx,gy,gz

#include <Wire.h>

#define MPU_ADDR 0x68
#define SDA_PIN 5
#define SCL_PIN 6
#define SAMPLE_INTERVAL_MS 20  // 50Hz

unsigned long t0 = 0;
unsigned long lastSampleMs = 0;
bool started = false;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  // 측정 범위를 시연 펌웨어(postureAlert.ino)와 맞춘다 — 기본값(±2g/±250°/s)은 충격·회전이 잘림
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);   // 가속도 범위 설정 레지스터
  Wire.write(0x18);   // ±16g
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);   // 자이로 범위 설정 레지스터
  Wire.write(0x18);   // ±2000°/s
  Wire.endTransmission();

  Serial.println("READY");
}

void readMpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // temperature, 사용 안 함
  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

  ax = ax_raw / 2048.0;   // g 단위 (±16g 기준: 2048 = 1g)
  ay = ay_raw / 2048.0;
  az = az_raw / 2048.0;
  gx = gx_raw / 16.4;     // deg/s (±2000°/s 기준: 16.4 = 1°/s)
  gy = gy_raw / 16.4;
  gz = gz_raw / 16.4;
}

void loop() {
  if (!started) {
    if (Serial.available() > 0 && Serial.read() == 's') {
      started = true;
      t0 = millis();
      lastSampleMs = t0;
    }
    return;
  }

  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs = now;

  float ax, ay, az, gx, gy, gz;
  readMpu(ax, ay, az, gx, gy, gz);

  Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                now - t0, ax, ay, az, gx, gy, gz);
}
