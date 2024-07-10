#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

MAX30105 particleSensor;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // 10085 for BMP180

void setup() {
  Serial.begin(115200);

  Serial.println("Starting setup...");

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1); // Stuck here if sensor not found
  }
  Serial.println("MAX30105 initialized.");
  particleSensor.setup(); // Configure MAX30105

  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }
  Serial.println("BMP180 initialized.");
}

void loop() {
  Serial.println("Entering loop...");

  // Read BMP180 sensor data
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    Serial.print("Pressure before IR check: ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
  } else {
    Serial.println("No pressure data available before IR check.");
  }

  // Read MAX30105 sensor data
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  if (irValue < 50000) {
    Serial.println("No finger detected.");
  } else {
    Serial.print("IR: ");
    Serial.print(irValue);
    Serial.print(" Red: ");
    Serial.println(redValue);

    // Read BMP180 sensor data again
    bmp.getEvent(&event);
    if (event.pressure) {
      Serial.print("Pressure after IR check: ");
      Serial.print(event.pressure);
      Serial.println(" hPa");
    } else {
      Serial.println("No pressure data available after IR check.");
    }

    // Prepare data for ESP8266 (or any other handling you want)
    Serial.print("Calling prepareDataForESP8266 with pressure: ");
    Serial.println(event.pressure);
    prepareDataForESP8266(irValue, redValue, event.pressure);
  }

  delay(1000); // Delay for a while before reading again
}

void prepareDataForESP8266(long irValue, long redValue, float pressure) {
  // Print values to check what is being received
  Serial.print("prepareDataForESP8266 received - IR: ");
  Serial.print(irValue);
  Serial.print(" Red: ");
  Serial.print(redValue);
  Serial.print(" Pressure: ");
  Serial.println(pressure);
}
