#include <Arduino.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_bt_device.h"

// =====================
// Switch and Pins
// =====================
constexpr int kSwitchPin = 6;
constexpr int kMmgSensorPins[3] = {19, 8, 4};
constexpr int kEmgSensorPin = 10;

constexpr int kSdaPin = 17;
constexpr int kSclPin = 18;

// =====================
// MPU6050 Registers
// =====================
constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kPwrMgmt1 = 0x6B;
constexpr uint8_t kAccelXoutH = 0x3B;

// =====================
// Sampling
// =====================
constexpr uint32_t kSampleIntervalMs = 10;
uint32_t lastSampleMs = 0;

struct ImuSample {
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
};

// =====================
// BLE Definitions
// =====================
#define SERVICE_UUID            "12a59900-17cc-11ec-9621-0242ac130002"
#define CHARACTERISTIC_UUID_RX  "12a59e0a-17cc-11ec-9621-0242ac130002"
#define CHARACTERISTIC_UUID_TX  "12a5a148-17cc-11ec-9621-0242ac130002"

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;

bool deviceConnected = false;
bool streaming = false;

bool lastSwitchState = HIGH;
uint32_t lastDebounceMs = 0;
constexpr uint32_t kDebounceDelayMs = 30;


// =====================
// MPU Functions
// =====================
void wakeMpu() {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(kPwrMgmt1);
  Wire.write(0x00);
  Wire.endTransmission();
}

ImuSample readImu() {
  ImuSample s{};

  Wire.beginTransmission(kMpuAddress);
  Wire.write(kAccelXoutH);
  Wire.endTransmission(false);
  Wire.requestFrom(kMpuAddress, (uint8_t)14);

  if (Wire.available() >= 14) {
    s.accelX = (Wire.read() << 8) | Wire.read();
    s.accelY = (Wire.read() << 8) | Wire.read();
    s.accelZ = (Wire.read() << 8) | Wire.read();
    s.gyroX  = (Wire.read() << 8) | Wire.read();
    s.gyroY  = (Wire.read() << 8) | Wire.read();
    s.gyroZ  = (Wire.read() << 8) | Wire.read();
  }
  return s;
}


// =====================
// BLE Callbacks
// =====================
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) override {
        deviceConnected = true;
        Serial.println("Device connected");
    }

    void onDisconnect(BLEServer *pServer) override {
        deviceConnected = false;
        Serial.println("Device disconnected");
        BLEDevice::startAdvertising();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            Serial.print("RX: ");
            Serial.println(rxValue.c_str());
        }
    }
};


// =====================
// Switch Debounce
// =====================
void handleSwitch() {
  int reading = digitalRead(kSwitchPin);

  if (reading != lastSwitchState) {
    lastDebounceMs = millis();
    lastSwitchState = reading;
  }

  if (millis() - lastDebounceMs < kDebounceDelayMs) return;

  bool switchOn = (reading == LOW);

  if (switchOn && !streaming) {
    streaming = true;
    Serial.println("Streaming started");
  }

  if (!switchOn && streaming) {
    streaming = false;
    Serial.println("Streaming stopped");
  }
}


// =====================
// Setup
// =====================
void setup() {
  Serial.begin(115200);

  pinMode(kSwitchPin, INPUT_PULLUP);
  for (int pin : kMmgSensorPins) pinMode(pin, INPUT);
  pinMode(kEmgSensorPin, INPUT);

  Wire.begin(kSdaPin, kSclPin);
  wakeMpu();

  // ===== BLE Init =====
  BLEDevice::init("SensorDataStreamer");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // TX notify characteristic
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX write characteristic
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(false);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising...");
}


// =====================
// Loop
// =====================
void loop() {
  handleSwitch();

  if (!streaming || !deviceConnected) return;

  uint32_t now = millis();
  if (now - lastSampleMs < kSampleIntervalMs) return;
  lastSampleMs = now;

  int mmg[3];
  for (int i = 0; i < 3; i++) mmg[i] = analogRead(kMmgSensorPins[i]);

  int emg = analogRead(kEmgSensorPin);
  ImuSample imu = readImu();

  char line[128];
  snprintf(line, sizeof(line),
    "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    (unsigned long)now,
    mmg[0], mmg[1], mmg[2],
    emg,
    imu.accelX, imu.accelY, imu.accelZ,
    imu.gyroX, imu.gyroY, imu.gyroZ
  );

  Serial.println(line);
  pTxCharacteristic->setValue(line);
  pTxCharacteristic->notify();
}
