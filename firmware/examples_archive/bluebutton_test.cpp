// firmware/src/test_servo.cpp
// Single-servo test - Feetech STS3215, ID 1 (default)
// STM32F446RE (Nucleo) — servo bus on PA9(TX)/PA10(RX), half-duplex TTL

#include <Arduino.h>

HardwareSerial ServoSerial(PA9);  // half-duplex, single pin

void sendServoPacket(uint8_t id, uint8_t instr, uint8_t* params, uint8_t paramLen) {
  uint8_t length = paramLen + 2;
  uint8_t checksum = id + length + instr;

  ServoSerial.write(0xFF);
  ServoSerial.write(0xFF);
  ServoSerial.write(id);
  ServoSerial.write(length);
  ServoSerial.write(instr);
  for (uint8_t i = 0; i < paramLen; i++) {
    ServoSerial.write(params[i]);
    checksum += params[i];
  }
  ServoSerial.write((uint8_t)(~checksum));

  ServoSerial.enableHalfDuplexRx();  // <-- CRITICAL: switch to RX after every TX
}

void writePos(uint8_t id, uint16_t pos, uint16_t time_ms, uint16_t speed) {
  uint8_t params[7] = {
    0x2A,                        // SMS_STS_GOAL_POSITION_L address
    (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
    (uint8_t)(time_ms & 0xFF), (uint8_t)(time_ms >> 8),
    (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)
  };
  sendServoPacket(id, 0x03 /* WRITE_DATA */, params, 7);
}

void queryPos(uint8_t id) {
  uint8_t params[2] = { 0x38 /* SMS_STS_PRESENT_POSITION_L */, 0x02 /* read len */ };
  sendServoPacket(id, 0x02 /* READ_DATA */, params, 2);

  // Give the servo time to respond, then relay raw bytes back to PC
  delay(5);
  while (ServoSerial.available()) {
    Serial.print(ServoSerial.read(), HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  ServoSerial.begin(1000000);
  delay(1000);
  Serial.println("READY - STM32 booted");  // <-- proves Serial TX works at all
}

void loop() {
if (Serial.available()) {
  char c = Serial.read();
  c = toupper(c);  // normalize to uppercase
  Serial.print("Got: ");
  Serial.println(c);

  switch (c) {
    case 'A': writePos(SERVO_ID, 0,    1000, 300); Serial.println("A: pos 0"); break;
    case 'B': writePos(SERVO_ID, 2048, 1000, 300); Serial.println("B: pos 2048 (center)"); break;
    case 'C': writePos(SERVO_ID, 4095, 1000, 300); Serial.println("C: pos 4095"); break;
    case 'H': writePos(SERVO_ID, 2048, 1000, 300); Serial.println("H: home"); break;
    case 'Q': Serial.println("Q: querying position..."); queryPos(SERVO_ID); break;
    default: break;
  }
}
}