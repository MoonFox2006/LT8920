#include <Arduino.h>
#include "LT8920.h"

//#define SENDER

const uint8_t LED_PIN = LED_BUILTIN;
const bool LED_LEVEL = LOW;

const uint8_t CS_PIN = D8;
const int8_t RST_PIN = -1;

const uint8_t CHANNEL = 102;

LT8920<CS_PIN, RST_PIN> lt;

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ! LED_LEVEL);

  lt.begin();
  lt.setChannel(CHANNEL);
  lt.setAutoAck(true);

#ifndef SENDER
  lt.listen();
#endif
}

void loop() {
#ifdef SENDER
  const uint32_t PERIOD = 1000; // 1 sec.

  char data[64];

  digitalWrite(LED_PIN, LED_LEVEL);
  lt.sleep(false);
  strcpy_P(data, PSTR("01234567890123456789012345678901234567890123456789*****"));
  ultoa(millis() / 1000, &data[55], 10);
  Serial.print(F("Sending \""));
  Serial.print(data);
  Serial.print(F("\" "));
  if (lt.send((uint8_t*)data, strlen(data)))
    Serial.println(F("OK"));
  else
    Serial.println(F("fail!"));
  lt.sleep(true);
  digitalWrite(LED_PIN, ! LED_LEVEL);
  delay(PERIOD);
#else
  if (lt.available()) {
    uint8_t buffer[64];
    int16_t size;

    digitalWrite(LED_PIN, LED_LEVEL);
    size = lt.read(buffer, sizeof(buffer) - 1);
    if (size <= 0) {
      Serial.println(F("Wrong packet length!"));
    } else {
      buffer[size] = '\0';
      Serial.print(F("Packet received: \""));
      Serial.print((char*)buffer);
      Serial.println('"');
    }
    lt.listen();
    digitalWrite(LED_PIN, ! LED_LEVEL);
  }
#endif
}
