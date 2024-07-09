#include <Wire.h>
#include "MAX30105.h"
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

MAX30105 particleSensor;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // 10085 for BMP180
SoftwareSerial esp8266(2, 3); // RX, TX

void setup() {
  Serial.begin(115200);
  esp8266.begin(115200);

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

  // Read MAX30105 sensor data
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Read BMP180 sensor data
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    Serial.print("Pressure: ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
  }

  if (irValue < 50000) {
    Serial.println("No finger?");
  } else {
    Serial.print("IR: ");
    Serial.print(irValue);
    Serial.print(" Red: ");
    Serial.println(redValue);

    // Prepare data for ESP8266
    prepareDataForESP8266(irValue, redValue, event.pressure);
  }

  delay(1000);
}

void prepareDataForESP8266(long irValue, long redValue, float pressure) {
  // Prepare sensor data
  String data = "IR: " + String(irValue) + ", Red: " + String(redValue);
  if (pressure != 0.0) {
    data += ", Pressure: " + String(pressure) + " hPa";
  }
  Serial.println("Data prepared for ESP8266: " + data);

  // Send data to ESP8266 (replace with actual ESP8266 communication)
  esp8266.println(data);
}
