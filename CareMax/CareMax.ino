#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// Constants for heart rate and temperature thresholds
const int NORMAL_HEART_RATE_MIN = 60;
const int NORMAL_HEART_RATE_MAX = 100;
const float HOT_TEMPERATURE_THRESHOLD = 30.0;
const float COLD_TEMPERATURE_THRESHOLD = 10.0;

// Initialize sensors
MAX30105 particleSensor;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // 10085 for BMP180

// Variables
unsigned long lastBeat = 0;
float beatsPerMinute;
float lastTemperature = 0.0;
bool measurementComplete = false;

void setup() {
  Serial.begin(115200);

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  // Initialize BMP180 sensor
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
      Serial.println("-----------------------------------------------------------------------------");
      Serial.println("|                                Results                                    |");
      Serial.println("-----------------------------------------------------------------------------");
      Serial.print("Heart rate: ");
      Serial.print(beatsPerMinute);
      Serial.println(" bpm");

      // Check if heart rate is within normal range for adults (60-100 bpm)
      if (beatsPerMinute >= NORMAL_HEART_RATE_MIN && beatsPerMinute <= NORMAL_HEART_RATE_MAX) {
        Serial.println("Status: Normal heart rate.");
      } else {
        Serial.println("Status: Abnormal heart rate."); // Handle abnormal cases here
        // Provide tips for abnormal heart rate
        if (beatsPerMinute < NORMAL_HEART_RATE_MIN) {
          Serial.println("Action: Your heart rate is lower than normal. Take deep breaths and relax.");
        } else if (beatsPerMinute > NORMAL_HEART_RATE_MAX) {
          Serial.println("Action: Your heart rate is higher than normal. Try to relax and calm down.");
        }
        // Implement additional actions as needed
      }

      // Read BMP180 temperature data
      float temperature;
      bmp.getTemperature(&temperature);
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");

      // Check temperature thresholds
      if (temperature > HOT_TEMPERATURE_THRESHOLD) {
        Serial.println("Status: Temperature is too hot! Alert!");
        Serial.println("Action: Move to a cooler place and drink water.");
        // Implement action for hot temperature
      } else if (temperature < COLD_TEMPERATURE_THRESHOLD) {
        Serial.println("Status: Temperature is too cold! Alert!");
        Serial.println("Action: Keep warm and avoid exposure to cold.");
        // Implement action for cold temperature
      } else {
        Serial.println("Status: Temperature is normal.");
      }

      // Display wait message after successful reading
      Serial.println("Note: Wait for 5 seconds to do another reading.");
      Serial.println("-----------------------------------------------------------------------------");

      // Set last temperature and measurement complete flag
      lastTemperature = temperature;
      measurementComplete = true;
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


