#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// === Pins ===
#define TX_PIN 14
#define RX_PIN 12

// === Manchester timing ===
#define BIT_DURATION_MS 100

// === Message à transmettre ===
const char* messageToSend = "Gab le quick";

// === Buffer de réception partagé ===
char receivedBuffer[256];

// === Constantes de trame ===
const uint8_t PREAMBLE = 0x55;
const uint8_t START_FLAG = 0x7E;
const uint8_t END_FLAG = 0x7E;

// === Fonctions utilitaires ===
uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (data[i] << 8);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void printByteBinary(uint8_t byte) {
  for (int i = 7; i >= 0; i--) Serial.print((byte >> i) & 0x01);
}

// === Fonctions Manchester ===
void sendManchesterBit(uint8_t bit) {
  if (bit == 0) {
    digitalWrite(TX_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(BIT_DURATION_MS / 2));
    digitalWrite(TX_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(BIT_DURATION_MS / 2));
  } else {
    digitalWrite(TX_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(BIT_DURATION_MS / 2));
    digitalWrite(TX_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(BIT_DURATION_MS / 2));
  }
}

uint8_t readManchesterBit() {
  int firstHalf = digitalRead(RX_PIN);
  vTaskDelay(pdMS_TO_TICKS(BIT_DURATION_MS / 2));
  int secondHalf = digitalRead(RX_PIN);
  vTaskDelay(pdMS_TO_TICKS(BIT_DURATION_MS / 2));

  if (firstHalf == HIGH && secondHalf == LOW) return 0;
  if (firstHalf == LOW && secondHalf == HIGH) return 1;

  return 0;
}

void sendManchesterByte(uint8_t byte) {
  for (int i = 0; i < 8; i++) {
    sendManchesterBit((byte >> i) & 0x01);
  }
}

uint8_t receiveManchesterByte() {
  uint8_t byte = 0;
  for (int i = 0; i < 8; i++) {
    byte |= (readManchesterBit() << i);
  }
  return byte;
}

void sendManchesterMessage(const char* msg) {
  size_t len = strlen(msg);
  uint16_t crc = crc16((const uint8_t*)msg, len);

  sendManchesterByte(PREAMBLE);
  sendManchesterByte(START_FLAG);
  sendManchesterByte(0x00); // type/flags
  sendManchesterByte(len);

  for (size_t i = 0; i < len; i++) {
    sendManchesterByte(msg[i]);
  }

  sendManchesterByte((crc >> 8) & 0xFF);
  sendManchesterByte(crc & 0xFF);
  sendManchesterByte(END_FLAG);

  // Silence après envoi pour laisser le récepteur se resynchroniser
  digitalWrite(TX_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(200));
}

bool receiveManchesterFrame(char* msgBuffer) {
  while (true) {
    uint8_t b = receiveManchesterByte();
    if (b == PREAMBLE) {
      b = receiveManchesterByte();
      if (b == START_FLAG) break;
    }
  }

  receiveManchesterByte(); // type/flags (ignoré ici)
  uint8_t msgLen = receiveManchesterByte();
  if (msgLen >= sizeof(receivedBuffer)) return false;

  for (int i = 0; i < msgLen; i++) {
    msgBuffer[i] = receiveManchesterByte();
  }

  uint16_t crcReceived = ((uint16_t)receiveManchesterByte() << 8) | receiveManchesterByte();
  uint8_t endFlag = receiveManchesterByte();

  if (endFlag != END_FLAG) return false;

  uint16_t calcCRC = crc16((const uint8_t*)msgBuffer, msgLen);
  if (crcReceived != calcCRC) return false;

  msgBuffer[msgLen] = '\0';
  return true;
}

// === TÂCHES ===
void taskSendMessage(void* pvParameters) {
  while (true) {
    sendManchesterMessage(messageToSend);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void taskReceiveMessage(void* pvParameters) {
  for (;;) {
    if (receiveManchesterFrame(receivedBuffer)) {
      Serial.print("[RX] Message reçu : ");
      Serial.println(receivedBuffer);
    }
    // Pas de délai ici pour permettre réception continue
  }
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT_PULLUP);
  digitalWrite(TX_PIN, HIGH);

  Serial.println("Démarrage des tâches sur 2 cœurs...");

  xTaskCreatePinnedToCore(taskSendMessage, "TaskTX", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskReceiveMessage, "TaskRX", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // FreeRTOS gère les tâches
}
