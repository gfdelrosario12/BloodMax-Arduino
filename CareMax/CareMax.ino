#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

MAX30105 particleSensor;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // 10085 for BMP180

unsigned long lastBeat = 0;
float beatsPerMinute;
bool measurementComplete = false;

void setup() {
  Serial.begin(115200);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read IR and Red values from MAX30105 sensor
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Calculate heart rate based on the pulsatile signal
  if (irValue < 50000) {
    // No finger detected or insufficient signal
    Serial.println("No finger detected or insufficient signal.");
    beatsPerMinute = 0; // Reset heart rate
    measurementComplete = false;
  } else {
    // Calculate heart rate if a valid signal is detected
    unsigned long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      // Print heart rate to serial monitor if valid
      Serial.print("Heart rate: ");
      Serial.print(beatsPerMinute);
      Serial.println(" bpm");

      // Read BMP180 temperature data
      float temperature;
      bmp.getTemperature(&temperature);
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");

      // Set measurement complete flag
      measurementComplete = true;
    } else {
      // Handle anomalous heart rate calculation
      Serial.println("Anomalous heart rate calculation detected.");
      beatsPerMinute = 0; // Reset heart rate
      measurementComplete = false;
    }
  }

  // If measurement is complete, pause for 5 seconds
  if (measurementComplete) {
    delay(5000); // 5 seconds delay
    measurementComplete = false; // Reset measurement flag
  } else {
    delay(1000); // Adjust delay as needed for your application
  }
}
