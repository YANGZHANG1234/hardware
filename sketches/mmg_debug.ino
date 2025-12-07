#include <Arduino.h>

// Pin assignments (3-channel MMG)
constexpr int kMmgSensorPins[3] = {19, 8, 4};

constexpr uint32_t kSampleIntervalMs = 10; // 100 Hz
uint32_t lastSample = 0;

void setup() {
  Serial.begin(115200);
  for (int pin : kMmgSensorPins) {
    pinMode(pin, INPUT);
  }
}

void loop() {
  const uint32_t now = millis();
  if (now - lastSample < kSampleIntervalMs) {
    return;
  }
  lastSample = now;

  Serial.print("MMG:");
  for (size_t i = 0; i < 3; ++i) {
    const int value = analogRead(kMmgSensorPins[i]);
    Serial.printf("%s%d", (i == 0 ? "" : ","), value);
  }
  Serial.println();
}
