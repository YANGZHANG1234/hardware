#include <Arduino.h>
#include <Wire.h>

// I2C pins for MPU6050 (modify if needed for your ESP32-S3 board)
constexpr int kSdaPin = 17;
constexpr int kSclPin = 18;

constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kPwrMgmt1 = 0x6B;
constexpr uint8_t kAccelXoutH = 0x3B;

constexpr uint32_t kSampleIntervalMs = 10; // 100 Hz
uint32_t lastSample = 0;

struct ImuSample {
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

void wakeMpu() {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(kPwrMgmt1);
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission();
}

ImuSample readImu() {
  ImuSample sample = {};

  Wire.beginTransmission(kMpuAddress);
  Wire.write(kAccelXoutH);
  Wire.endTransmission(false);
  Wire.requestFrom(kMpuAddress, static_cast<uint8_t>(14));

  if (Wire.available() >= 14) {
    sample.accelX = (Wire.read() << 8) | Wire.read();
    sample.accelY = (Wire.read() << 8) | Wire.read();
    sample.accelZ = (Wire.read() << 8) | Wire.read();
    sample.gyroX  = (Wire.read() << 8) | Wire.read();
    sample.gyroY  = (Wire.read() << 8) | Wire.read();
    sample.gyroZ  = (Wire.read() << 8) | Wire.read();
  }

  return sample;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(kSdaPin, kSclPin);

  wakeMpu();
  Serial.println("MPU6050 IMU Test Started");
}

void loop() {
  const uint32_t now = millis();
  if (now - lastSample < kSampleIntervalMs) {
    return;
  }
  lastSample = now;

  ImuSample sample = readImu();

  Serial.printf(
    "ACC:%d,%d,%d  GYRO:%d,%d,%d\n",
    sample.accelX, sample.accelY, sample.accelZ,
    sample.gyroX, sample.gyroY, sample.gyroZ
  );
}
