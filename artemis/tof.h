#include <SparkFun_VL53L5CX_Library.h>

class ToF {
  SparkFun_VL53L5CX sensor;
  VL53L5CX_ResultsData data;
public:
  ToF();
  void begin();
  void read();
  VL53L5CX_ResultsData* get_data();
};
