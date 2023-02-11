#include "motor.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Motor::begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  Motor::Left::forward(100);
  delay(500);
  Motor::Left::stop();
  delay(500);
  Motor::Left::backward(100);
  delay(500);
  Motor::Left::stop();
  delay(500);
}
