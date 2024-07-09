#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_BMP085.h>  // Library for BMP180 sensor
#include "MAX30105.h"         // Sparkfun MAX3010X library
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <WiFiEspUdp.h>
#include <SoftwareSerial.h>

MAX30105 particleSensor;                      // Initialize the MAX30105 sensor object
Adafruit_BMP085 bmp;                          // Initialize the BMP180 sensor object

// WiFi credentials
char ssid[] = "ZTE_2.4G_sYFQcE";
char pass[] = "YYXPYK9";

int status = WL_IDLE_STATUS;  // WiFi status
SoftwareSerial esp8266(2, 3); // RX, TX

// Variables for storing and calculating sensor data
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;  // Sampling interval for calculating SpO2
int Temperature;
int temp;
float ESpO2;  // Initial value of estimated SpO2
float ESpO2_ROM;
double FSpO2 = 0.7;   // Filter factor for estimated SpO2
double frate = 0.95;  // Low pass filter for IR/red LED value to eliminate AC component

// Constants for timing and scaling
#define TIMETOBOOT 3000  // Time to wait before outputting SpO2 (in milliseconds)
#define SCALE 88.0       // Scaling factor for displaying heart beat and SpO2
#define SAMPLING 5       // Sampling interval for more precise heart beat
#define FINGER_ON 30000  // Threshold to indicate finger is on the sensor
#define USEFIFO
#define Greenled 8  // Pin for green LED
#define Redled 9    // Pin for red LED

void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud rate
  esp8266.begin(115200); // Start software serial for ESP8266

  ESpO2 = readEEPROM();          // Read stored SpO2 value from EEPROM
  Temperature = EEPROM.read(6);  // Read stored temperature value from EEPROM

  pinMode(Greenled, OUTPUT);    // Set green LED pin as output
  pinMode(Redled, OUTPUT);      // Set red LED pin as output
  digitalWrite(Greenled, LOW);  // Turn off green LED
  digitalWrite(Redled, LOW);    // Turn off red LED

  // Initialize MAX30102 sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper.");
  }

  // Set up MAX30102 sensor parameters
  byte ledBrightness = 0x7F;  // LED brightness (0-255)
  byte sampleAverage = 4;     // Number of samples to average (1, 2, 4, 8, 16, 32)
  byte ledMode = 2;           // LED mode (1 = Red only, 2 = Red + IR, 3 = Red + IR + Green)
  int sampleRate = 200;       // Sample rate (50, 100, 200, 400, 800, 1000, 1600, 3200)
  int pulseWidth = 411;       // Pulse width (69, 118, 215, 411)
  int adcRange = 16384;       // ADC range (2048, 4096, 8192, 16384)

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();  // Enable temperature readings from the sensor

  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {}  // Infinite loop if sensor is not found
  }
}

void loop() {
  uint32_t ir, red;  // Variables to store sensor readings
  double fred, fir;  // Variables to store converted sensor readings
  double SpO2 = 0;   // Raw SpO2 before low pass filtering

#ifdef USEFIFO
  particleSensor.check();  // Check the sensor and read up to 3 samples

  while (particleSensor.available()) {  // If new data is available
    red = particleSensor.getFIFOIR();   // Get IR data (note: this may actually be red data for MAX30102)
    ir = particleSensor.getFIFORed();   // Get red data (note: this may actually be IR data for MAX30102)

    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);  // Average red level with low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate);     // Average IR level with low pass filter
    sumredrms += (fred - avered) * (fred - avered);         // Sum of squared differences for red level
    sumirrms += (fir - aveir) * (fir - aveir);              // Sum of squared differences for IR level

    if ((i % SAMPLING) == 0) {  // Throttle the graph plotting speed
      if (millis() > TIMETOBOOT) {
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;      // Normalize and scale IR data
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;  // Normalize and scale red data

        // Truncate data for Serial plotter's autoscaling
        if (ir_forGraph > 100.0) ir_forGraph = 100.0;
        if (ir_forGraph < 80.0) ir_forGraph = 80.0;
        if (red_forGraph > 100.0) red_forGraph = 100.0;
        if (red_forGraph < 80.0) red_forGraph = 80.0;

        if (ir < FINGER_ON) {       // If finger is not on the sensor
          temp = Temperature;       // Store current temperature
          EEPROM.write(6, temp);    // Save temperature to EEPROM
          Serial.print("Temperature:");
          Serial.print(temp);
          Serial.println("C");

          Serial.print("EsPo2 = ");
          Serial.println(ESpO2);
          delay(500);  // Delay for a short period
        }
        Serial.print(int(ir_forGraph));  // Output normalized IR data for plotting
        Serial.print(",");
        Serial.print(int(red_forGraph));  // Output normalized red data for plotting
        Serial.print(",");
        Serial.print(ESpO2);  // Output estimated SpO2
        Serial.println("");
      }
    }

    if ((i % Num) == 0) {
      sumredrms = sqrt(sumredrms / Num);  // Calculate RMS of red data
      sumirrms = sqrt(sumirrms / Num);    // Calculate RMS of IR data

      // Avoid division by zero and update estimated SpO2
      if (sumirrms > 0 && sumredrms > 0) {
        SpO2 = 100.0 - 5.0 * (sumredrms / sumirrms) * (avered / aveir) * FSpO2;  // Calculate raw SpO2
      }

      // Apply low pass filter to estimated SpO2
      if (SpO2 > 99.0) SpO2 = 99.0;
      if (SpO2 < 80.0) SpO2 = 80.0;
      ESpO2 = (ESpO2 * frate + SpO2 * (1.0 - frate));
      ESpO2_ROM = ESpO2;
      writeEEPROM(ESpO2_ROM);
      sumredrms = 0.0;
      sumirrms = 0.0;
      Serial.print("Temperature:");
      temp = Temperature;
      Serial.print(temp);
      Serial.println("C");
    }
  }

  if (ir < FINGER_ON) {  // If finger is not on the sensor
    digitalWrite(Greenled, LOW);
    digitalWrite(Redled, HIGH);
    Serial.println("Please Put Your Finger on Sensor!");
    Serial.print("EsPO2% = ");
    Serial.println(ESpO2);
    Serial.print("temp = ");
    Serial.println(temp);
  }

  if (ir > FINGER_ON) {  // If finger is on the sensor
    digitalWrite(Greenled, HIGH);
    digitalWrite(Redled, LOW);
  }

  particleSensor.nextSample();  // Advance to the next sample in the FIFO
#endif
}

float readEEPROM() {
  float value = 0.0;
  EEPROM.get(0, value);  // Read a float value from EEPROM address 0
  return value;
}

void writeEEPROM(float value) {
  EEPROM.put(0, value);  // Write a float value to EEPROM address 0
}
