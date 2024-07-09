#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include <EEPROM.h>
#include <Adafruit_BMP085.h>  // Library for BMP180 sensor
#include "MAX30105.h"         // Sparkfun MAX3010X library
#include <ESP8266WiFi.h>      // Placeholder for ESP8266 library

MAX30105 particleSensor;                      // Initialize the MAX30105 sensor object
Adafruit_BMP085 bmp;                          // Initialize the BMP180 sensor object
LiquidCrystal_I2C lcd(0x27, 20, 4);           // Initialize the LCD display object
Adafruit_MLX90614 mlx = Adafruit_MLX90614();  // Initialize the MLX90614 sensor object

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

  lcd.init();                  // Initialize the LCD
  lcd.backlight();             // Turn on the LCD backlight
  lcd.setCursor(3, 1);         // Set cursor position on LCD
  lcd.print("Running......");  // Display initial message
  delay(3000);                 // Wait for 3 seconds
  lcd.clear();                 // Clear the LCD

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
  mlx.begin();                        // Initialize the MLX90614 sensor

  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {}  // Infinite loop if sensor is not found
  }

  // Placeholder for ESP8266 setup
  // WiFi.begin(ssid, password);
}

void loop() {
  uint32_t ir, red, green;  // Variables to store sensor readings
  double fred, fir;         // Variables to store converted sensor readings
  double SpO2 = 0;          // Raw SpO2 before low pass filtering

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
          ESpO2_ROM = ESpO2;        // Store current SpO2
          writeEEPROM(&ESpO2_ROM);  // Save SpO2 to EEPROM

          // Display last test results on the LCD
          lcd.setCursor(0, 2);
          lcd.print("Last test: ");
          lcd.setCursor(11, 2);
          lcd.print(ESpO2_ROM);
          lcd.print(" %");

          lcd.setCursor(0, 3);
          lcd.print("Last temp: ");
          lcd.setCursor(11, 3);
          lcd.print(temp);
          lcd.print(" *C");
          break;
        }

        if (ir > FINGER_ON) {                   // If finger is on the sensor
          Temperature = mlx.readObjectTempC();  // Read temperature from MLX90614

          // Display current SpO2 and temperature on the LCD
          lcd.setCursor(0, 0);
          lcd.print("Oxygen % = ");
          lcd.setCursor(11, 0);
          lcd.print(ESpO2);
          lcd.print(" %");

          lcd.setCursor(0, 1);
          lcd.print("Temperature: ");
          lcd.print(Temperature);
          lcd.print(" *C");

          // Read and display pressure from BMP180 sensor
          float pressure = bmp.readPressure();
          lcd.setCursor(0, 2);
          lcd.print("Pressure: ");
          lcd.print(pressure / 100.0);  // Convert pressure to hPa
          lcd.print(" hPa");

          // Read and display altitude from BMP180 sensor
          float altitude = bmp.readAltitude();
          lcd.setCursor(0, 3);
          lcd.print("Altitude: ");
          lcd.print(altitude);
          lcd.print(" m");

          // Control LEDs based on SpO2 and temperature values
          if ((ESpO2 >= 90) && (Temperature < 38)) {
            digitalWrite(Redled, LOW);     // Turn off red LED
            digitalWrite(Greenled, HIGH);  // Turn on green LED
          } else {
            digitalWrite(Greenled, LOW);  // Turn off green LED
            digitalWrite(Redled, HIGH);   // Turn on red LED
          }
        }
      }
    }

    if ((i % Num) == 0) {                                                // Calculate SpO2 every Num samples
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);  // Calculate R ratio
      SpO2 = -23.3 * (R - 0.4) + 100;                                    // Calculate SpO2 based on R ratio
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;                      // Apply low pass filter to SpO2
      sumredrms = 0.0;                                                   // Reset sum of squared differences for red level
      sumirrms = 0.0;                                                    // Reset sum of squared differences for IR level
      i = 0;                                                             // Reset sample counter
      break;
    }
    particleSensor.nextSample();  // Move to the next sample
  }
#endif
}

// Function to write SpO2 value to EEPROM
void writeEEPROM(float *data) {
  byte ByteArray[4];           // Array to hold byte representation of float
  memcpy(ByteArray, data, 4);  // Copy float data to byte array
  for (int x = 0; x < 4; x++) {
    EEPROM.write(x, ByteArray[x]);  // Write each byte to EEPROM
  }
}

// Function to read SpO2 value from EEPROM
float readEEPROM() {
  float ESpO2 = 85.0;  // Default SpO2 value if EEPROM is not initialized
  byte ByteArray[4];   // Array to hold byte representation of float
  for (int x = 0; x < 4; x++) {
    ByteArray[x] = EEPROM.read(x);  // Read each byte from EEPROM
  }
  memcpy(&ESpO2, ByteArray, 4);  // Copy byte array to float
  return ESpO2;                  // Return the read SpO2 value
}