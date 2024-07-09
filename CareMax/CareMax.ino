#include <Wire.h>
#include "MAX30105.h"
#include <SoftwareSerial.h>

MAX30105 particleSensor;

SoftwareSerial esp8266(2, 3); // RX, TX

void setup() {
  Serial.begin(115200);
  esp8266.begin(115200);

  Serial.println("Starting setup...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1); // Stuck here if sensor not found
  }
  
  Serial.println("MAX30105 initialized.");
  
  particleSensor.setup(); // Configure sensor with default settings
  Serial.println("Particle sensor setup complete.");
}

void loop() {
  Serial.println("Entering loop...");
  
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  
  if (irValue < 50000) {
    Serial.println("No finger?");
  } else {
    Serial.print("IR: ");
    Serial.print(irValue);
    Serial.print(" Red: ");
    Serial.println(redValue);

    // Placeholder for sending data to the ESP8266
    prepareDataForESP8266(irValue, redValue);
  }
  
  delay(1000);
}

void prepareDataForESP8266(long irValue, long redValue) {
  // Placeholder function to show where data would be processed
  String data = "IR: " + String(irValue) + ", Red: " + String(redValue);
  Serial.println("Data prepared for ESP8266: " + data);

  // Example of sending data to ESP8266 (currently just prints to Serial)
  esp8266.println(data);
}
