#include <SparkFun_VL53L5CX_Library.h>

class ToF {
public:
  SparkFun_VL53L5CX sensor;
  VL53L5CX_ResultsData data;

  ToF();
  void begin();
  void read();
};
