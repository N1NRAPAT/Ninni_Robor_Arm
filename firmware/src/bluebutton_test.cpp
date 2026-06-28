#include <Arduino.h>

#define DEBUG_SERIAL  Serial
#define DEBUG_BAUD    115200
#define BTN_PIN       PC13  // blue user button (active LOW)

void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  pinMode(BTN_PIN, INPUT);
  delay(500);

  DEBUG_SERIAL.println("=== Blue Button Test ===");
  DEBUG_SERIAL.println("Press blue button to send SNAP");
  DEBUG_SERIAL.println("=======================");
}

bool lastButtonState = HIGH;

void loop() {
  bool buttonState = digitalRead(BTN_PIN);
  if (lastButtonState == HIGH && buttonState == LOW) {  // falling edge
    DEBUG_SERIAL.println("SNAP");
    delay(50);  // debounce
  }
  lastButtonState = buttonState;
}
