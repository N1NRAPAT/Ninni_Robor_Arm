#include <Arduino.h>

HardwareSerial ServoSerial(PA9);
#define SERVO_ID 1

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

  ServoSerial.enableHalfDuplexRx();
}

void writePos(uint8_t id, uint16_t pos, uint16_t time_ms, uint16_t speed) {
  uint8_t params[7] = {
    0x2A,
    (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
    (uint8_t)(time_ms & 0xFF), (uint8_t)(time_ms >> 8),
    (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)
  };
  sendServoPacket(id, 0x03, params, 7);
}

void queryPos(uint8_t id) {
  uint8_t params[2] = { 0x38, 0x02 };
  sendServoPacket(id, 0x02, params, 2);

  delay(10);
  while (ServoSerial.available()) {
    Serial.print(ServoSerial.read(), HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("STEP 1 - Serial OK");

  ServoSerial.begin(1000000);
  Serial.println("STEP 2 - ServoSerial OK");

  delay(500);
  Serial.println("READY - STM32 booted (half-duplex)");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    c = toupper(c);
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
