#include <MPU9250.h>

class IMU {
public:
  MPU9250 sensor;

  void begin(TwoWire&, uint32_t); // upload calibrated metrics
  void read(); // filter stuff goes here

};
