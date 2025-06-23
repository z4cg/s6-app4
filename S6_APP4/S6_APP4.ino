#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// === Pins ===
#define TX_PIN 33
#define RX_PIN 25

// === Manchester timing ===
#define HALF_BIT_US 500

// === Message à transmettre ===
const char* messageToSend = "HELLO";

// === Buffer de réception partagé ===
char receivedBuffer[32];

// === Fonctions Manchester ===
void sendManchesterChar(char c) {
  for (int i = 7; i >= 0; i--) {
    bool bit = (c >> i) & 0x01;
    digitalWrite(TX_PIN, bit ? 1 : 0);
    delayMicroseconds(HALF_BIT_US);
    digitalWrite(TX_PIN, bit ? 0 : 1);
    delayMicroseconds(HALF_BIT_US);
  }
}

void sendManchesterMessage(const char* msg) {
  size_t len = strlen(msg);
  uint16_t crc = crc16((const uint8_t*)msg, len);

  /*Serial.print("[TX] Trame brute : ");
  printByteBinary(0x55); Serial.print(" ");
  printByteBinary(0x7E); Serial.print(" ");
  printByteBinary(0x02); Serial.print(" ");
  printByteBinary(len); Serial.print(" ");
  for (size_t i = 0; i < len; i++) {
    printByteBinary((uint8_t)msg[i]); Serial.print(" ");
  }
  printByteBinary((crc >> 8) & 0xFF); Serial.print(" ");
  printByteBinary(crc & 0xFF); Serial.print(" ");
  printByteBinary(0x7E);
  Serial.println();*/

  // Envoi réel
  sendManchesterChar(0x55);
  sendManchesterChar(0x7E);
  sendManchesterChar(0x02);
  sendManchesterChar(len);
  for (size_t i = 0; i < len; i++) {
    sendManchesterChar(msg[i]);
  }
  sendManchesterChar((crc >> 8) & 0xFF);
  sendManchesterChar(crc & 0xFF);
  sendManchesterChar(0x7E);
}

char receiveManchesterChar() {
  char c = 0;
  for (int i = 7; i >= 0; i--) {
    bool first = digitalRead(RX_PIN);
    delayMicroseconds(HALF_BIT_US);
    bool second = digitalRead(RX_PIN);
    delayMicroseconds(HALF_BIT_US);

    if (first == 0 && second == 1) {
      c |= (1 << i);
    } else if (first == 1 && second == 0) {
      c &= ~(1 << i);
    }
  }
  return c;
}

bool receiveManchesterFrame(char* msgBuffer) {
  bool msgReceived = false;
  Serial.print("Hello");
  while (receiveManchesterChar() != 0x55){
    delayMicroseconds(HALF_BIT_US);
  }

  char start = receiveManchesterChar();
  Serial.print("Start: "); printByteBinary(start); Serial.print("\n");
  char typeFlag = receiveManchesterChar();
  Serial.print("typeFlag: "); printByteBinary(typeFlag); Serial.print("\n");
  int msgLen = (uint8_t)receiveManchesterChar();
  Serial.print("msgLen: "); printByteBinary(msgLen); Serial.print("\n");

  if (msgLen >= sizeof(receivedBuffer)) {
    Serial.println("[RX] Erreur: message trop long");
    msgBuffer[0] = '\0';
    return false;
  }
  for (int i = 0; i < msgLen; i++) {
    msgBuffer[i] = receiveManchesterChar();
  }
  uint8_t crcHigh = (uint8_t)receiveManchesterChar();  // bits 15 à 8
  uint8_t crcLow  = (uint8_t)receiveManchesterChar();  // bits 7 à 0
  uint16_t crc = ((uint16_t)crcHigh << 8) | crcLow;
  char end = receiveManchesterChar();

  if (start == 0x7E && end == 0x7E) {
    uint16_t calcCRC = crc16((const uint8_t*)msgBuffer, msgLen);
    if (crc == calcCRC) {
      msgReceived = true;
    } else {
      Serial.println("[RX] Erreur: CRC invalide");
    }
  } else {
    Serial.println("[RX] Erreur: start ou end invalide");
  }

  msgBuffer[msgLen] = '\0';
  Serial.print("Fin analyse trame");
  return msgReceived;
}

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void printByteBinary(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    Serial.print((byte >> i) & 0x01);
  }
}

void printBufferBinary(const char* buffer, int length) {
  for (int i = 0; i < length; i++) {
    printByteBinary((uint8_t)buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
}


// === TÂCHES ===

// Tâche 1 : émetteur (Core 0)
void taskSendMessage(void* pvParameters) {
  for (;;) {
    //Serial.println("[TX] Envoi du message...");
    sendManchesterMessage(messageToSend);
    vTaskDelay(pdMS_TO_TICKS(3000)); // attend 3 secondes
  }
}

// Tâche 2 : récepteur (Core 1)
void taskReceiveMessage(void* pvParameters) {
  for (;;) {
    Serial.println("[RX] Attente d’un message...");
    if (receiveManchesterFrame(receivedBuffer)) {
      Serial.print("[RX] Message reçu : ");
      Serial.println(receivedBuffer);
    } else {
      Serial.println("[RX] Trame invalide ou erreur de réception.");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);

  Serial.println("Démarrage des tâches sur 2 cœurs...");

  // Crée la tâche d’émission sur Core 0
  xTaskCreatePinnedToCore(
    taskSendMessage,      // fonction
    "TaskTX",             // nom pour debug
    2048,                 // stack size (en mots, pas octets)
    NULL,                 // paramètre
    1,                    // priorité
    NULL,                 // handle (non utilisé)
    0                     // Core 0
  );

  // Crée la tâche de réception sur Core 1
  xTaskCreatePinnedToCore(
    taskReceiveMessage,
    "TaskRX",
    2048,
    NULL,
    1,
    NULL,
    1                    // Core 1
  );
}

void loop() {
  // Vide – FreeRTOS prend le relais
}