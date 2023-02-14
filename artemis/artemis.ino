#include <Wire.h>
#include "tof.h"
#include "imu.h"

#define I2C_FREQ 400000

TwoWire i2c(D25, D27);
ToF tof;
IMU imu;

void setup() {

  Serial.begin(115200);
  while(!Serial);

  i2c.begin();
  i2c.setClock(I2C_FREQ);

  tof.begin();
  imu.begin(i2c, I2C_FREQ);
}

void loop() {
  tof.read();
  Serial.print(tof.data.distance_mm[3]);
  Serial.println();
  delay(3);
}
