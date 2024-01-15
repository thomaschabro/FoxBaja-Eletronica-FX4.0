// Importing libraries
#include <ESP32-TWAI-CAN.hpp>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <SoftwareSerial.h>

// Defines for ESP32 CAN Bus communication
#define CAN_TX		5
#define CAN_RX		4

// Defines for LoRa communication
#define LoRa_MOSI 10
#define LoRa_MISO 11
#define LoRa_SCK 9

#define LoRa_nss 8
#define LoRa_dio1 14
#define LoRa_nrst 12
#define LoRa_busy 13

SX1262 radio = new Module(LoRa_nss, LoRa_dio1, LoRa_nrst, LoRa_busy);

CanFrame rxFrame;
int potencia1 = 0;
int velocidade = 0;

void setup() {
  // Setup serial for debbuging.
  Serial.begin(115200);

  // Set pins
  ESP32Can.setPins(CAN_TX, CAN_RX);
  
  // You can set custom size for the queues - those are default
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);

  // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
  // but you can easily convert it from numerical value using .convertSpeed()
  ESP32Can.setSpeed(ESP32Can.convertSpeed(125));

  // You can also just use .begin()..
  if(ESP32Can.begin()) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }

  // or override everything in one command;
  // It is also safe to use .begin() without .end() as it calls it internally
  if(ESP32Can.begin(ESP32Can.convertSpeed(125), CAN_TX, CAN_RX, 10, 10)) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }

  // ===============  SETUP FOR LORA COMMUNICATION =================================
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_nss);

  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true)
      ;
  }
}

void loop() {
    // static uint32_t lastStamp = 0;
    // uint32_t currentStamp = millis();
    
    // if(currentStamp - lastStamp > 1000) {   // sends OBD2 request every second
    //     lastStamp = currentStamp;
    //     sendObdFrame(5); // For coolant temperature
    // }


    // ============ CAN RECEIVING PACKET ================
    // You can set custom timeout, default is 1000
    Serial.printf("Waiting for packet\n");
    if(ESP32Can.readFrame(rxFrame, 1000)) {
        // Comment out if too many frames
        Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
        if(rxFrame.identifier == 0x0B1) {   // Standard OBD2 frame responce ID

          uint32_t data1 = 0;
          for (int i = 0; i < 4; i++) {
              data1 |= (rxFrame.data[i] << (i * 8));
          }
          potencia1 = (int) data1;
          Serial.printf("Received data1 value: %d \n", potencia1);
          Serial.print(" = \n");

        }
    }

    // ============ SENDING VALUE BY LORA ==============
    String lora_message = "Valor : " + String(potencia1);
    Serial.print("Enviando a mensagem: " + lora_message + "\n");
    int state = radio.transmit(lora_message);
}