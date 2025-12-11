#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// Adjust chip select pin for your board; GPIO5 is common on ESP32-S3
constexpr int kSdCsPin = 5;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  Serial.println("Initializing SD card...");
  if (!SD.begin(kSdCsPin)) {
    Serial.println("SD card init failed. Check wiring and CS pin.");
    return;
  }

  Serial.printf("Card type: %d\n", SD.cardType());
  Serial.printf("Card size: %llu MB\n", SD.cardSize() / (1024 * 1024));

  File testFile = SD.open("/sd_test.txt", FILE_WRITE);
  if (!testFile) {
    Serial.println("Failed to open sd_test.txt for writing.");
    return;
  }

  testFile.println("SD card write test successful.");
  testFile.close();
  Serial.println("Write complete; reading file back...");

  testFile = SD.open("/sd_test.txt", FILE_READ);
  if (!testFile) {
    Serial.println("Failed to reopen sd_test.txt for reading.");
    return;
  }

  while (testFile.available()) {
    Serial.write(testFile.read());
  }
  testFile.close();
  Serial.println("\nSD card test finished.");
}

void loop() {
  // Nothing to do in loop for this simple SD card test
}
