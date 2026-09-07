// Holy-IOT iBeacon(UUID: FDA50693-A4E2-4FB1-AFCF-C6EB07647825)만 걸러서
// major/minor/RSSI를 파싱하고, 신호가 가장 센 비콘을 현재 구역으로 판단한다.
// 독립 실행용 진단 스케치 — postureAlert.ino엔 이 로직이 FreeRTOS 태스크로 이식되어 있음.

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define SCAN_TIME_SEC 3

// 우리 비콘들의 공통 iBeacon UUID (raw byte 순서 그대로, MFG 데이터에서 읽은 순서)
const uint8_t TARGET_UUID[16] = {
  0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1,
  0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25
};

struct BeaconReading {
  uint16_t major;
  uint16_t minor;
  int rssi;
};

BLEScan* pBLEScan;
BeaconReading best;
bool foundAny;

bool parseIBeacon(const uint8_t* mfg, size_t len, uint16_t &major, uint16_t &minor) {
  // 최소 길이: companyId(2) + type(1) + len(1) + uuid(16) + major(2) + minor(2) + txPower(1) = 25
  if (len < 25) return false;
  if (mfg[2] != 0x02 || mfg[3] != 0x15) return false;  // iBeacon type/length 마커
  for (int i = 0; i < 16; i++) {
    if (mfg[4 + i] != TARGET_UUID[i]) return false;
  }
  major = (mfg[20] << 8) | mfg[21];
  minor = (mfg[22] << 8) | mfg[23];
  return true;
}

class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (!advertisedDevice.haveManufacturerData()) return;

    String md = advertisedDevice.getManufacturerData();
    if (md.length() < 25) return;

    uint16_t major, minor;
    if (!parseIBeacon((const uint8_t*)md.c_str(), md.length(), major, minor)) return;

    int rssi = advertisedDevice.getRSSI();
    Serial.printf("  [비콘] major=%u minor=%u rssi=%d\n", major, minor, rssi);

    if (!foundAny || rssi > best.rssi) {
      best = {major, minor, rssi};
      foundAny = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("타겟 비콘 스캔 시작 (Holy-IOT UUID 필터링)");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallback(), true);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
}

void loop() {
  foundAny = false;
  Serial.println("--- 스캔 라운드 ---");
  pBLEScan->start(SCAN_TIME_SEC, false);
  pBLEScan->clearResults();

  if (foundAny) {
    Serial.printf(">>> 현재 구역 추정: major=%u minor=%u (rssi=%d)\n\n",
                  best.major, best.minor, best.rssi);
  } else {
    Serial.println(">>> 타겟 비콘 없음\n");
  }
  delay(300);
}
