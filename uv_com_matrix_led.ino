#include <SoftwareSerial.h>
#include <Arduino.h> // Include the Arduino library for NVIC_SystemReset()
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#define TFT_CS        13 // pin CS
#define TFT_DC         9 // pin Data/Command
#define TFT_RST        8 // pin Reset, o -1 se collegato al pin reset  Arduino

const int rxPin = 6;
const int txPin = 7;
const int buzzerPin = 5;

SoftwareSerial co2Serial(rxPin, txPin);

int co2Level = 0;

int32_t uv1 = 0;
int32_t uv2 = 0;

// Initialize the Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Variáveis para controle de tempo e estado
bool co2ReadingInProgress = false;
unsigned long co2RequestTime = 0;
const unsigned long co2Interval = 5000; // Intervalo de tempo entre leituras de CO2 (5 segundos)
const unsigned long co2Timeout = 10000; // Tempo limite para a leitura de CO2 (10 segundos)

// Limites para o alarme do UV e CO2
const int uvMin = 300; // Nível mínimo de UV para alarme
const int uvMax = 800; // Nível máximo de UV para alarme
const int co2Min = 800; // Nível mínimo de CO2 para alarme
const int co2Max = 2000; // Nível máximo de CO2 para alarme

// Configuração do buzzer
const int minPulseDuration = 100; // Duração mínima do pulso do buzzer em ms
const int maxPulseDuration = 500; // Duração máxima do pulso do buzzer em ms
const int minVolume = 50; // Volume mínimo do buzzer (0-255)
const int maxVolume = 255; // Volume máximo do buzzer (0-255)
const unsigned long alarmInterval = 5000; // Intervalo para o alarme do buzzer em ms

// Variáveis para armazenar os valores antigos
int lastCo2Level = -1;
int lastMaxUV = -1;

// Variáveis para controle do estado do buzzer
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
int currentPulseDuration = 0;
int currentVolume = 0;
unsigned long lastAlarmTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  co2Serial.begin(9600);

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

  // Inicializa o display com SPI_MODE2
  tft.init(240, 240, SPI_MODE2);
  delay(500);  // Adiciona atraso após a inicialização

  tft.setRotation(3); // Gira o display 90 graus
  tft.fillScreen(ST77XX_BLACK);
}

void loop() {
  uv1 = analogRead(A0);
  uv2 = analogRead(A1);
  int32_t maxUV = max(uv1, uv2);

  // Verifica se é hora de enviar um novo comando de leitura de CO2
  if (!co2ReadingInProgress && millis() - co2RequestTime >= co2Interval) {
    requestCO2Reading();
    co2ReadingInProgress = true;
    co2RequestTime = millis();
  }

  // Tenta ler a resposta do sensor de CO2 se a leitura estiver em andamento
  if (co2ReadingInProgress) {
    int result = readCO2NonBlocking();
    if (result != -1) {
      co2Level = result;
      Serial.print("CO2 Level: ");
      Serial.println(co2Level);
      co2ReadingInProgress = false; // Reseta o estado da leitura
    }
    // Reinicia a leitura de CO2 se não houver resposta em 10 segundos
    else if (millis() - co2RequestTime >= co2Timeout) {
      Serial.println("CO2 sensor not responding, resetting...");
      co2ReadingInProgress = false; // Cancela a leitura em andamento
      requestCO2Reading(); // Reinicia a leitura
      co2RequestTime = millis(); // Atualiza o tempo do último pedido
    }
  }

  // Atualiza o display com as leituras atuais
  updateDisplay(maxUV, co2Level);

  // Controla o buzzer baseado nos níveis de UV e CO2
  controlBuzzer(maxUV, co2Level);

  // Print the readings and the maximum value
  // Serial.print("A0: ");
  // Serial.print(uv1);
  // Serial.print("\tA1: ");
  // Serial.print(uv2);
  // Serial.print("\tMax UV: ");
  // Serial.println(maxUV);

  // Delay para permitir a leitura do próximo ciclo sem bloqueio
  delay(20);
}

void requestCO2Reading() {
  // Envia o comando para solicitar a leitura de CO2
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
  // Verifica se há dados disponíveis para leitura
  if (co2Serial.available() >= 9) {
    byte response[9];
    co2Serial.readBytes(response, 9);

    if (response[0] == 0xFF && response[1] == 0x86) {
      return (response[2] << 8) + response[3];
    }
  }
  return -1; // Retorna -1 se a leitura não estiver completa ou válida
}

void updateDisplay(int maxUV, int co2Level) {
  tft.setTextSize(9); // Ajusta o tamanho do texto
  tft.setFont(); // Usa a fonte padrão do Adafruit GFX

  // Atualiza a exibição do nível de CO2 somente se mudou
  if (co2Level != lastCo2Level) {
    tft.setCursor(5, 30); // Ajusta a posição para centralizar verticalmente
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Define a cor do texto e o fundo preto
    tft.print(lastCo2Level); // Apaga o número antigo desenhando o fundo
    tft.setCursor(5, 30); // Reajusta o cursor após apagar
    tft.print(co2Level); // Desenha o novo número
    lastCo2Level = co2Level; // Atualiza o valor antigo de CO2
  }

  // Atualiza a exibição do nível de UV somente se mudou
  if (maxUV != lastMaxUV) {
    tft.setCursor(5, 150); // Ajusta a posição para centralizar verticalmente
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK); // Define a cor do texto e o fundo preto

    // Converte maxUV em uma string com 4 caracteres
    char uvString[5]; // Cria um buffer para a string
    snprintf(uvString, sizeof(uvString), "%-4d", maxUV); // Formata maxUV para ser uma string de 4 caracteres

    tft.print(lastMaxUV); // Apaga o número antigo desenhando o fundo
    tft.setCursor(5, 150); // Reajusta o cursor após apagar
    tft.print(uvString); // Desenha o novo número formatado
    lastMaxUV = maxUV; // Atualiza o valor antigo de UV
  }
}



void controlBuzzer(int maxUV, int co2Level) {
  // Verifica se os níveis de UV ou CO2 estão acima dos limites mínimos
  bool uvAlert = maxUV > uvMin;
  bool co2Alert = co2Level > co2Min;

  if ((uvAlert || co2Alert) && millis() - lastAlarmTime >= alarmInterval) {
    // Calcula a duração do pulso com base nos níveis de UV e CO2
    int uvPulseDuration = uvAlert ? map(maxUV, uvMin, uvMax, minPulseDuration, maxPulseDuration) : 0;
    int co2PulseDuration = co2Alert ? map(co2Level, co2Min, co2Max, minPulseDuration, maxPulseDuration) : 0;
    currentPulseDuration = max(uvPulseDuration, co2PulseDuration);

    // Calcula o volume do buzzer com base nos níveis de UV e CO2
    int uvVolume = uvAlert ? map(maxUV, uvMin, uvMax, minVolume, maxVolume) : 0;
    int co2Volume = co2Alert ? map(co2Level, co2Min, co2Max, minVolume, maxVolume) : 0;
    currentVolume = max(uvVolume, co2Volume);

    if (!buzzerActive) {
      // Inicia o alarme
      buzzerStartTime = millis();
      analogWrite(buzzerPin, currentVolume); // Define o volume do buzzer
      buzzerActive = true;
      lastAlarmTime = millis(); // Atualiza o tempo do último alarme
      // Serial.print("Activating Buzzer - Duration: ");
      // Serial.print(currentPulseDuration);
      // Serial.print(" ms, Volume: ");
      // Serial.println(currentVolume);
    } else if (millis() - buzzerStartTime >= currentPulseDuration) {
      // Desliga o buzzer após a duração do pulso
      analogWrite(buzzerPin, 0);
      buzzerActive = false;
    }
  } else {
    analogWrite(buzzerPin, 0); // Garante que o buzzer está desligado
    buzzerActive = false;
  }
}
