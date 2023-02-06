#include "geometry_msgs/Quaternion.h"
#include "robot/RawImu.h"

void imu_callback(const robot::RawImu&);

void MadgwickQuaternionUpdate(float, float, float, float, float, float, float, float, float, float);
