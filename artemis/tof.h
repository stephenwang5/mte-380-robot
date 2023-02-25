#ifndef TOF_H
#define TOF_H

#include <SparkFun_VL53L5CX_Library.h>

class ToF {
public:
  SparkFun_VL53L5CX sensor;
  VL53L5CX_ResultsData data;
  int filterResult; // objective direction wrt the robot
  bool objectiveFound;

  ToF();
  void begin();
  void read();
  void filter();
};

#endif // TOF_H
