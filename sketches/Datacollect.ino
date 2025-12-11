#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Switch to control logging (active LOW with internal pull-up)
constexpr int kSwitchPin = 6;

// SD card SPI pins (adjust to match your board wiring)
constexpr int kSdCsPin   = 12;
constexpr int kSdClkPin  = 13;
constexpr int kSdMisoPin = 11;
constexpr int kSdMosiPin = 14;

// Pin assignments (3-channel MMG)
constexpr int kMmgSensorPins[3] = {19, 8, 4};

// Pin assignment for single EMG channel
constexpr int kEmgSensorPin = 10;

// I2C pins for MPU6050
constexpr int kSdaPin = 17;
constexpr int kSclPin = 18;

// MPU6050 register map
constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kPwrMgmt1 = 0x6B;
constexpr uint8_t kAccelXoutH = 0x3B;

// Sampling settings
constexpr uint32_t kSampleIntervalMs = 10; // 100 Hz
uint32_t lastSampleMs = 0;

struct ImuSample {
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

File logFile;
bool logging = false;
int fileIndex = 1;
bool lastSwitchState = HIGH;
uint32_t lastDebounceMs = 0;
constexpr uint32_t kDebounceDelayMs = 30;

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

String nextFilePath() {
  // Files are named /1.csv, /2.csv, /3.csv ...
  while (SD.exists("/" + String(fileIndex) + ".csv")) {
    ++fileIndex;
  }
  return "/" + String(fileIndex) + ".csv";
}

bool startLogging() {
  const String path = nextFilePath();
  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) {
    Serial.printf("Failed to open %s for writing\n", path.c_str());
    return false;
  }

  Serial.printf("Logging started: %s\n", path.c_str());
  logFile.println(
    "timestamp_ms,mmg1,mmg2,mmg3,emg,accelX,accelY,accelZ,gyroX,gyroY,gyroZ"
  );
  logging = true;
  lastSampleMs = millis();
  return true;
}

void stopLogging() {
  if (logFile) {
    logFile.flush();
    logFile.close();
    Serial.println("Logging stopped and file saved.");
  }
  logging = false;
  ++fileIndex;  // Prepare for the next file number
}

void handleSwitch() {
  const int reading = digitalRead(kSwitchPin);
  if (reading != lastSwitchState) {
    lastDebounceMs = millis();
    lastSwitchState = reading;
  }

  if ((millis() - lastDebounceMs) < kDebounceDelayMs) {
    return;  // Ignore bouncing
  }

  // Active LOW: LOW means switch ON
  const bool switchOn = (reading == LOW);
  if (switchOn && !logging) {
    startLogging();
  } else if (!switchOn && logging) {
    stopLogging();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(kSwitchPin, INPUT_PULLUP);
  for (int pin : kMmgSensorPins) {
    pinMode(pin, INPUT);
  }
  pinMode(kEmgSensorPin, INPUT);

  // Initialise SPI bus and SD card
  SPI.begin(kSdClkPin, kSdMisoPin, kSdMosiPin, kSdCsPin);
  if (!SD.begin(kSdCsPin, SPI)) {
    Serial.println("SD card init failed. Check wiring and CS pin.");
    return;
  }
  Serial.println("SD card ready.");

  Wire.begin(kSdaPin, kSclPin);
  wakeMpu();
  Serial.println("MPU6050 initialized. Flip the switch to start logging.");
}

void loop() {
  handleSwitch();

  if (!logging) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastSampleMs < kSampleIntervalMs) {
    return;
  }
  lastSampleMs = now;

  // Read MMG values
  int mmgValues[3];
  for (size_t i = 0; i < 3; ++i) {
    mmgValues[i] = analogRead(kMmgSensorPins[i]);
  }

  // Read EMG value
  const int emgValue = analogRead(kEmgSensorPin);

  // Read IMU values
  ImuSample imu = readImu();

  // Write combined data line
  logFile.printf(
    "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    static_cast<unsigned long>(now),
    mmgValues[0], mmgValues[1], mmgValues[2],
    emgValue,
    imu.accelX, imu.accelY, imu.accelZ,
    imu.gyroX, imu.gyroY, imu.gyroZ
  );
}
