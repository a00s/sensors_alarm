#include <SoftwareSerial.h>
// #include "Arduino_LED_Matrix.h"
#include <Arduino.h> // Include the Arduino library for NVIC_SystemReset()
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
 
#define TFT_CS        13 // pin CS
#define TFT_DC         9 // pin Data/Command
#define TFT_RST        8 // pin Reset, o -1 se collegato al pin reset  Arduino

// ArduinoLEDMatrix matrix;

const int numReadings = 5;
int readingsA0[numReadings];
int readingsA1[numReadings];
int readIndex = 0;

const int rxPin = 6;
const int txPin = 7;
const int buzzerPin = 5;

SoftwareSerial co2Serial(rxPin, txPin);

int co2Level = 0;

const int numRows = 8;
const int numCols = 12;

// Define thresholds for UV and CO2
const int uvMin = 40;
const int uvMax = 200;
const int co2Min = 600;
const int co2Max = 2000;

// Define minimum and maximum pulse duration and volume
const int minPulseDuration = 10; // Minimum duration of beep in ms
const int maxPulseDuration = 500; // Maximum duration of beep in ms
const int minVolume = 50; // Minimum volume (50% duty cycle)
const int maxVolume = 255; // Maximum volume (100% duty cycle)

uint8_t frame[numRows][numCols] = {0};
int maxLevel = 20;

// Counter for consecutive failed CO2 readings
const int maxFailedReadings = 5;
int failedReadingsCount = 0;
// int maxUV = 200;

// Initializza la Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  co2Serial.begin(9600);
  // matrix.begin();

  // Buzzer test
  Serial.println("Testing buzzer...");
  analogWrite(buzzerPin, 70); // 50% duty cycle to reduce volume
  delay(40);
  analogWrite(buzzerPin, 0);
  delay(40);
  analogWrite(buzzerPin, 70); // 50% duty cycle to reduce volume
  delay(40);
  analogWrite(buzzerPin, 0);
  delay(40);
  analogWrite(buzzerPin, 70); // 50% duty cycle to reduce volume
  delay(40);
  analogWrite(buzzerPin, 0);


  delay(1000);
  Serial.println("Buzzer test complete.");

  for (int i = 0; i < numReadings; i++) {
    readingsA0[i] = 0;
    readingsA1[i] = 0;
  }

// Inicializa o display com SPI_MODE2
  tft.init(240, 240, SPI_MODE2);
  delay(500);  // Adiciona atraso após a inicialização


  tft.setRotation(3); //ruota di 180°
 
  tft.fillScreen(ST77XX_BLACK);
 
  // tft.setTextSize(3);   //Imposta la grandezza del testo
  // tft.setTextColor(ST77XX_WHITE);  //Imposta il colore del testo
  // tft.setCursor(10, 10);
  // tft.println("TEST DISPLAY");  //Stringa da visualizzare
 
  // tft.setCursor(20, 50);
  // tft.setTextColor(ST77XX_RED);  //Imposta il colore del testo
  // tft.println("IPS 240x240");  //Stringa da visualizzare
 
  // tft.setCursor(20, 100);
  // tft.setTextColor(ST77XX_YELLOW);  //Imposta il colore del testo
  // tft.println("Arduino UNO");  //Stringa da visualizzare
 
  // tft.setCursor(20, 150);
  // tft.setTextSize(2);
  // tft.setTextColor(ST77XX_GREEN);  //Imposta il colore del testo
  // tft.println("AEEEEE"); //Stringa da visualizzare

}

void loop() {
  // Read the current value from the sensor
  readingsA0[readIndex] = analogRead(A0);
  readingsA1[readIndex] = analogRead(A1);
  
  readIndex = readIndex + 1;
  
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // Get the maximum value from the UV sensors
  int maxUV = max(readingsA0[readIndex], readingsA1[readIndex]);
  
  // Read CO2 level from MH-Z19B
  co2Level = readCO2();
   delay(5000);
  // Check if CO2 reading failed
  if (co2Level == -1) {
    failedReadingsCount++;
    if (failedReadingsCount >= maxFailedReadings) {
      Serial.println("Too many consecutive CO2 reading failures, resetting...");
      NVIC_SystemReset(); // Perform a system reset
    }
  } else {
    failedReadingsCount = 0; // Reset the counter if a valid reading is received
  }

  // Print the readings and the maximum value
  Serial.print("A0: ");
  Serial.print(readingsA0[readIndex]);
  Serial.print("\tA1: ");
  Serial.print(readingsA1[readIndex]);
  Serial.print("\tMax UV: ");
  Serial.print(maxUV);
  Serial.print("\tCO2: ");
  Serial.println(co2Level);
 
  updateDisplay(maxUV, co2Level);
 
  // Define the number of LEDs to light directly from the UV sensor reading
  // int ledsToLightUV = map((long)maxUV, 0, (long)maxLevel, 0, (long)(numRows * (numCols / 2)));
  int ledsToLightUV = map((long)maxUV, 0, (long)maxLevel, 0, (long)(numRows * (numCols / 2)));
  // Define the number of LEDs to light directly from the CO2 sensor reading
  int ledsToLightCO2 = map((long)co2Level, 0, 2000, 0, (long)(numRows * (numCols / 2)));
  
  // Update the frame of the LED matrix based on the sensor readings
  // updateLedMatrix(ledsToLightUV, ledsToLightCO2);

  // Render the frame on the LED matrix
  // matrix.renderBitmap(frame, numRows, numCols);

  // Check if thresholds are exceeded and activate the buzzer with pulsing frequency
  controlBuzzer(maxUV, co2Level);

  // Wait 2 seconds before taking the next reading
  delay(2000);
}

void controlBuzzer(int maxUV, int co2Level) {
  bool uvAlert = maxUV > uvMin;
  bool co2Alert = co2Level > co2Min;

  if (uvAlert || co2Alert) {
    int uvPulseDuration = uvAlert ? map(maxUV, uvMin, uvMax, minPulseDuration, maxPulseDuration) : 0;
    int co2PulseDuration = co2Alert ? map(co2Level, co2Min, co2Max, minPulseDuration, maxPulseDuration) : 0;
    int pulseDuration = max(uvPulseDuration, co2PulseDuration);

    int uvVolume = uvAlert ? map(maxUV, uvMin, uvMax, minVolume, maxVolume) : 0;
    int co2Volume = co2Alert ? map(co2Level, co2Min, co2Max, minVolume, maxVolume) : 0;
    int volume = max(uvVolume, co2Volume);

    Serial.print("Pulse Duration: ");
    Serial.print(pulseDuration);
    Serial.print(" ms, Volume: ");
    Serial.println(volume);
    analogWrite(buzzerPin, volume); // Set volume
    delay(pulseDuration); // Duration of beep
    analogWrite(buzzerPin, 0);
    delay(100); // Delay between beeps
  } else {
    analogWrite(buzzerPin, 0); // Ensure the buzzer is off
  }
}

// void updateLedMatrix(int ledsToLightUV, int ledsToLightCO2) {
//   memset(frame, 0, sizeof(frame));

//   int count = 0;
//   for (int row = 0; row < numRows; row++) {
//     for (int col = 0; col < numCols / 2; col++) {
//       if (count < ledsToLightUV) {
//         frame[row][col] = 1;
//         count++;
//       } else {
//         frame[row][col] = 0;
//       }
//     }
//   }

//   count = 0;
//   for (int row = 0; row < numRows; row++) {
//     for (int col = numCols / 2; col < numCols; col++) {
//       if (count < ledsToLightCO2) {
//         frame[row][col] = 1;
//         count++;
//       } else {
//         frame[row][col] = 0;
//       }
//     }
//   }
// }

int readCO2() {
  byte response[9];
  const int maxRetries = 5;
  int retries = 0;
  int co2Value = -1;

  while (retries < maxRetries && co2Value == -1) {
    co2Serial.write((uint8_t)0xFF);
    co2Serial.write((uint8_t)0x01);
    co2Serial.write((uint8_t)0x86);
    co2Serial.write((uint8_t)0x00);
    co2Serial.write((uint8_t)0x00);
    co2Serial.write((uint8_t)0x00);
    co2Serial.write((uint8_t)0x00);
    co2Serial.write((uint8_t)0x00);
    co2Serial.write((uint8_t)0x79);

    memset(response, 0, 9);
    co2Serial.readBytes(response, 9);

    if (response[0] == 0xFF && response[1] == 0x86) {
      co2Value = (response[2] << 8) + response[3];
    } else {
      retries++;
      delay(100);
    }
  }

  return co2Value;
}

void updateDisplay(int maxUV, int co2Level) {
  tft.setTextSize(9);
  // Clear the area where CO2 level is displayed
  tft.fillRect(0, 0, 240, 240, ST77XX_BLACK); // Adjust width and height as needed

  // Set text color and display new CO2 level
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 10);
  tft.println(co2Level);

  // Clear the area where UV level is displayed
  // tft.fillRect(10, 70, 200, 30, ST77XX_BLACK); // Adjust width and height as needed

  // Set text color and display new UV level
  tft.setTextColor(ST77XX_RED);
  tft.setCursor(10, 100);
  tft.println(maxUV);
}
