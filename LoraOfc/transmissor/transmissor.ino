/*
  Code Base from RadioLib: https://github.com/jgromes/RadioLib/tree/master/examples/SX126x

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <SoftwareSerial.h>

#define LoRa_MOSI 10
#define LoRa_MISO 11
#define LoRa_SCK 9

#define LoRa_nss 8
#define LoRa_dio1 14
#define LoRa_nrst 12
#define LoRa_busy 13

SX1262 radio = new Module(LoRa_nss, LoRa_dio1, LoRa_nrst, LoRa_busy);

// Definindo pinos que serão emulados como RX e TX
const int rxPin = 47;
const int txPin = 48;

SoftwareSerial mySerial(rxPin, txPin);

void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // Emulando a serial
  mySerial.begin(115200);
  SoftwareSerial mySerial(rxPin, txPin);  // RX, TX

  Serial.begin(115200);
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
  if (mySerial.available() > 0) {
    Serial.println("Chegou pacote");
    // Armazenar em uma variável o que foi recebido
    char texto[20];
    texto[0] = 0;
    int i = 0;
    while (mySerial.available()) {
      byte c = mySerial.read();
      texto[i] = c;
      i++;
      Serial.print(c);
    }
    Serial.println(texto);
  } else {
    Serial.println("Nothing available");
  }

  // Serial.print(F("[SX1262] Transmitting packet ... "));

  int state = radio.transmit("Hello World!");

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    //Serial.println(F("success!"));

    // print measured data rate
    //Serial.print(F("[SX1262] Datarate:\t"));
    //Serial.print(radio.getDataRate());
    //Serial.println(F(" bps"));
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    //Serial.println(F("too long!"));
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    //Serial.println(F("timeout!"));
  } else {
    // some other error occurred
    //Serial.print(F("failed, code "));
    //Serial.println(state);
  }

  // wait for a second before transmitting again
  delay(1000);
}