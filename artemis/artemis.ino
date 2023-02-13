#include <Wire.h>
#include "tof.h"

ToF tof;

void setup() {

  Serial.begin(115200);
  while(!Serial);

  Wire.begin();
  Wire.setClock(400000); // 400 kHz

  tof.begin();
}

void loop() {
  tof.read();
  Serial.print(tof.get_data()->distance_mm[3]);
  Serial.println();
  delay(3);
}
