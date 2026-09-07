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
// 실측 확인된 배포 비콘의 제조사 ID (little-endian 그대로, MFG 데이터 앞 2바이트).
// UUID만으로도 사실상 충돌 가능성은 없지만, 프레임 마커를 전부 검증하는 게 맞아서 추가.
const uint8_t TARGET_COMPANY_ID[2] = {0xFF, 0xFF};

// 구역 전환 hysteresis: 새 비콘이 기존 구역보다 이만큼(dB) 더 세게 잡혀야 갈아탄다.
// 신호가 비슷한 두 비콘 사이에서 매 라운드 왔다갔다(flicker)하는 걸 막기 위함
// (실측 중 실제로 관찰된 현상).
const int ZONE_SWITCH_HYSTERESIS_DB = 5;

struct BeaconReading {
  uint16_t major;
  uint16_t minor;
  int rssi;
};

BLEScan* pBLEScan;
BeaconReading roundBest;      // 이번 스캔 라운드에서 가장 센 비콘
bool foundAny;
BeaconReading currentBest;    // 라운드 간 유지되는, hysteresis 적용된 현재 구역
bool haveCurrentBest = false;

bool parseIBeacon(const uint8_t* mfg, size_t len, uint16_t &major, uint16_t &minor) {
  // 최소 길이: companyId(2) + type(1) + len(1) + uuid(16) + major(2) + minor(2) + txPower(1) = 25
  if (len < 25) return false;
  if (mfg[0] != TARGET_COMPANY_ID[0] || mfg[1] != TARGET_COMPANY_ID[1]) return false;
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

    if (!foundAny || rssi > roundBest.rssi) {
      roundBest = {major, minor, rssi};
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
    // 아직 확정된 구역이 없거나, 같은 비콘이거나, 기존 구역보다 hysteresis만큼
    // 더 세게 잡혀야만 구역을 갈아탄다 — 신호가 비슷할 때 매 라운드 뒤바뀌는 걸 방지.
    bool shouldSwitch = !haveCurrentBest
        || roundBest.major == currentBest.major
        || roundBest.rssi > currentBest.rssi + ZONE_SWITCH_HYSTERESIS_DB;
    if (shouldSwitch) {
      currentBest = roundBest;
      haveCurrentBest = true;
    }
    Serial.printf(">>> 현재 구역 추정: major=%u minor=%u (rssi=%d, 이번 라운드 최고=%u/%d)\n\n",
                  currentBest.major, currentBest.minor, currentBest.rssi,
                  roundBest.major, roundBest.rssi);
  } else {
    Serial.println(">>> 타겟 비콘 없음\n");
  }
  delay(300);
}
