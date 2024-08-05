#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "MHZ19.h"

#define TFT_CS        13 // pin CS
#define TFT_DC         9 // pin Data/Command
#define TFT_RST        8 // pin Reset, o -1 se collegato al pin reset  Arduino

const int rxPin = 6;
const int txPin = 7;
const int buzzerPin = 5;

MHZ19 myMHZ19;  
SoftwareSerial co2Serial(rxPin, txPin);

int co2Level = 0;
int maxCo2Level = 0; // To track maximum CO2 level

int32_t uv1 = 0;
int32_t uv2 = 0;
int maxUVLevel = 0; // To track maximum UV level

// Initialize the Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Variables for time control and state
bool co2ReadingInProgress = false;
unsigned long co2RequestTime = 0;
const unsigned long co2Interval = 5000; // Interval between CO2 readings (5 seconds)
const unsigned long co2Timeout = 10000; // Timeout for CO2 reading (10 seconds)
unsigned long graphUpdateTime = 0; // Time control for graph update
const unsigned long graphInterval = 2000; // Interval for graph update (2 seconds)

// Alarm limits for UV and CO2
const int uvMin = 300; // Minimum UV level for alarm
const int uvMax = 800; // Maximum UV level for alarm
const int co2Min = 800; // Minimum CO2 level for alarm
const int co2Max = 2000; // Maximum CO2 level for alarm

// Buzzer configuration
const int minPulseDuration = 100; // Minimum buzzer pulse duration in ms
const int maxPulseDuration = 500; // Maximum buzzer pulse duration in ms
const int minVolume = 50; // Minimum buzzer volume (0-255)
const int maxVolume = 255; // Maximum buzzer volume (0-255)
const unsigned long alarmInterval = 5000; // Interval for buzzer alarm in ms

// Variables to store old values
int lastCo2Level = -1;
int lastMaxUV = -1;

// Variables to control buzzer state
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
int currentPulseDuration = 0;
int currentVolume = 0;
unsigned long lastAlarmTime = 0;

// Buffers to store historical values for graph
const int bufferLength = 120; // Half the width of the display
int co2Buffer[bufferLength];
int uvBuffer[bufferLength]; // Buffer for UV

// Fail counter for CO2 sensor
int co2FailCount = 0;
const int maxFailCount = 5; // Number of consecutive failures before restart

void myMHZ19_start(){
  co2Serial.begin(9600);                               // (Uno example) device to MH-Z19 serial start
  myMHZ19.begin(co2Serial);                                // *Serial(Stream) refence must be passed to library begin().
}

void setup() {
  Serial.begin(9600);
  // co2Serial.begin(9600);
  // co2Serial.begin(Serial);

  pinMode(buzzerPin, OUTPUT);
  
  // Buzzer test
  Serial.println("Testing buzzer...");
  analogWrite(buzzerPin, 70); // 50% duty cycle to reduce volume
  delay(40);
  analogWrite(buzzerPin, 0);
  delay(40);
  analogWrite(buzzerPin, 70); // 50% duty cycle to reduce volume
  delay(40);
  analogWrite(buzzerPin, 0);
  analogWrite(buzzerPin, 70); // 50% duty cycle to reduce volume
  delay(40);
  analogWrite(buzzerPin, 0);

  delay(1000);
  Serial.println("Buzzer test complete.");

  // Initialize display with SPI_MODE2
  tft.init(240, 240, SPI_MODE2);
  delay(500);  // Add delay after initialization

  tft.setRotation(3); // Rotate display 90 degrees
  tft.fillScreen(ST77XX_BLACK);

  // Initialize buffers to zero
  for (int i = 0; i < bufferLength; i++) {
    co2Buffer[i] = 0;
    uvBuffer[i] = 0; // Initialize UV buffer
  }

    //CO2
  myMHZ19_start();
  myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))
  // co2Serial.end();										  //close MH-Z19 COM port

  // Request initial CO2 sensor reading
  requestCO2Reading();
  co2ReadingInProgress = true;
  co2RequestTime = millis();
}

void loop() {
  uv1 = analogRead(A0);
  uv2 = analogRead(A1);
  int32_t maxUV = max(uv1, uv2);
  
  // Update the maximum UV value
  if (maxUV > maxUVLevel) {
    maxUVLevel = maxUV;
  }

  // Check if it's time to send a new CO2 reading command
  if (!co2ReadingInProgress && millis() - co2RequestTime >= co2Interval) {
    requestCO2Reading();
    co2ReadingInProgress = true;
    co2RequestTime = millis();
  }

  // Try to read the CO2 sensor response if reading is in progress
  if (co2ReadingInProgress) {
    int result = readCO2NonBlocking();
    if (result != -1) {
      co2Level = result;
      if (co2Level > maxCo2Level) { // Update maximum CO2 if necessary
        maxCo2Level = co2Level;
      }
      Serial.print("CO2 Level: ");
      Serial.println(co2Level);
      co2ReadingInProgress = false; // Reset reading state
      co2FailCount = 0; // Reset fail counter on success
    }
    // Restart reading if there is no response in 10 seconds
    else if (millis() - co2RequestTime >= co2Timeout) {
      Serial.println("CO2 sensor not responding, resetting...");
      co2ReadingInProgress = false; // Cancel ongoing reading
      co2FailCount++; // Increment fail counter
      requestCO2Reading(); // Restart reading
      co2RequestTime = millis(); // Update last request time
    }

    // Restart if fail count exceeds the threshold
    if (co2FailCount > maxFailCount) {
      Serial.println("Maximum consecutive CO2 sensor failures reached. Restarting...");
      delay(1000); // Delay for observation
      restartArduino();
    }
  }

  // Update display with current readings
  updateDisplay(maxUV, co2Level);

  // Update graphs every 2 seconds
  if (millis() - graphUpdateTime >= graphInterval) {
    updateCO2Graph(co2Level);
    updateUVGraph(maxUV);
    graphUpdateTime = millis();
  }

  // Control buzzer based on UV and CO2 levels
  controlBuzzer(maxUV, co2Level);

  // Delay to allow the next cycle to read without blocking
  delay(20);
}

void requestCO2Reading() {
  // Send command to request CO2 reading
  co2Serial.write((uint8_t)0xFF);
  co2Serial.write((uint8_t)0x01);
  co2Serial.write((uint8_t)0x86);
  co2Serial.write((uint8_t)0x00);
  co2Serial.write((uint8_t)0x00);
  co2Serial.write((uint8_t)0x00);
  co2Serial.write((uint8_t)0x00);
  co2Serial.write((uint8_t)0x00);
  co2Serial.write((uint8_t)0x79);
}

int readCO2NonBlocking() {
  // Check if data is available for reading
  if (co2Serial.available() >= 9) {
    byte response[9];
    co2Serial.readBytes(response, 9);

    if (response[0] == 0xFF && response[1] == 0x86) {
      return (response[2] << 8) + response[3];
    }
  }
  return -1; // Return -1 if the reading is not complete or valid
}

void updateDisplay(int maxUV, int co2Level) {
  tft.setTextSize(4); // Increase text size for CO2 and UV value
  tft.setFont(); // Use default Adafruit GFX font

  // Upper left quadrant - CO2
  if (co2Level != lastCo2Level) {
    tft.setCursor(5, 5); // Set position in the upper left quadrant
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Set text and background color
    tft.fillRect(0, 0, 120, 60, ST77XX_BLACK); // Clear area before drawing new number
    tft.setCursor(5, 5); // Reset cursor after clearing
    tft.print(co2Level); // Draw new number
    lastCo2Level = co2Level; // Update old CO2 value
  }

  // Lower left quadrant - UV
  if (maxUV != lastMaxUV) {
    tft.setCursor(5, 125); // Set position in the lower left quadrant
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK); // Set text and background color
    tft.fillRect(0, 120, 120, 60, ST77XX_BLACK); // Clear area before drawing new number
    tft.setCursor(5, 125); // Reset cursor after clearing
    tft.print(maxUV); // Draw new number
    lastMaxUV = maxUV; // Update old UV value
  }
}

void updateCO2Graph(int co2Level) {
  // Shift old values to the left
  for (int i = 0; i < bufferLength - 1; i++) {
    co2Buffer[i] = co2Buffer[i + 1];
  }
  // Add new value at the end of the buffer
  co2Buffer[bufferLength - 1] = co2Level;

  // Clear the CO2 graph area
  tft.fillRect(120, 0, 120, 120, ST77XX_BLACK);

  // Draw the CO2 graph in the upper right quadrant
  for (int i = 1; i < bufferLength; i++) {
    int co2Y1 = map(co2Buffer[i - 1], 0, maxCo2Level, 119, 0); // Map based on maximum value read
    int co2Y2 = map(co2Buffer[i], 0, maxCo2Level, 119, 0);
    tft.drawLine(120 + i - 1, co2Y1, 120 + i, co2Y2, ST77XX_WHITE);
  }
}

void updateUVGraph(int maxUV) {
  // Shift old values to the left
  for (int i = 0; i < bufferLength - 1; i++) {
    uvBuffer[i] = uvBuffer[i + 1];
  }
  // Add new value at the end of the buffer
  uvBuffer[bufferLength - 1] = maxUV;

  // Clear the UV graph area
  tft.fillRect(120, 120, 120, 120, ST77XX_BLACK);

  // Draw the UV graph in the lower right quadrant
  for (int i = 1; i < bufferLength; i++) {
    int uvY1 = map(uvBuffer[i - 1], 0, maxUVLevel, 239, 120); // Map based on maximum value read
    int uvY2 = map(uvBuffer[i], 0, maxUVLevel, 239, 120);
    tft.drawLine(120 + i - 1, uvY1, 120 + i, uvY2, ST77XX_RED);
  }
}

void controlBuzzer(int maxUV, int co2Level) {
  // Check if UV or CO2 levels exceed minimum limits
  bool uvAlert = maxUV > uvMin;
  bool co2Alert = co2Level > co2Min;

  if ((uvAlert || co2Alert) && millis() - lastAlarmTime >= alarmInterval) {
    // Calculate pulse duration based on UV and CO2 levels
    int uvPulseDuration = uvAlert ? map(maxUV, uvMin, uvMax, minPulseDuration, maxPulseDuration) : 0;
    int co2PulseDuration = co2Alert ? map(co2Level, co2Min, co2Max, minPulseDuration, maxPulseDuration) : 0;
    currentPulseDuration = max(uvPulseDuration, co2PulseDuration);

    // Calculate buzzer volume based on UV and CO2 levels
    int uvVolume = uvAlert ? map(maxUV, uvMin, uvMax, minVolume, maxVolume) : 0;
    int co2Volume = co2Alert ? map(co2Level, co2Min, co2Max, minVolume, maxVolume) : 0;
    currentVolume = max(uvVolume, co2Volume);

    if (!buzzerActive) {
      // Start the alarm
      buzzerStartTime = millis();
      analogWrite(buzzerPin, currentVolume); // Set buzzer volume
      buzzerActive = true;
      lastAlarmTime = millis(); // Update last alarm time
    } else if (millis() - buzzerStartTime >= currentPulseDuration) {
      // Turn off the buzzer after pulse duration
      analogWrite(buzzerPin, 0);
      buzzerActive = false;
    }
  } else {
    analogWrite(buzzerPin, 0); // Ensure the buzzer is off
    buzzerActive = false;
  }
}

void restartArduino() {
  // Trigger a software reset by jumping to the start address
  NVIC_SystemReset();  // For ARM Cortex-M boards
}
