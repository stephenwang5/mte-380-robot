#include <quaternionFilters.h>
#include "imu.h"

void IMU::begin(TwoWire& bus, uint32_t freq) {
  sensor = MPU9250(MPU9250_ADDRESS_AD0, bus, freq);
  sensor.initMPU9250();
  sensor.initAK8963(sensor.factoryMagCalibration);

  sensor.getAres();
  sensor.getGres();
  sensor.getMres();
  // calibrate with sensor.magCalMPU9250()
  
}

void IMU::read() {
  sensor.readAccelData(sensor.accelCount);
  sensor.ax = (float)sensor.accelCount[0] * sensor.aRes; // - sensor.accelBias[0];
  sensor.ay = (float)sensor.accelCount[1] * sensor.aRes; // - sensor.accelBias[1];
  sensor.az = (float)sensor.accelCount[2] * sensor.aRes; // - sensor.accelBias[2];

  sensor.readGyroData(sensor.gyroCount);  // Read the x/y/z adc code
  sensor.gx = (float)sensor.gyroCount[0] * sensor.gRes;
  sensor.gy = (float)sensor.gyroCount[1] * sensor.gRes;
  sensor.gz = (float)sensor.gyroCount[2] * sensor.gRes;

  sensor.readMagData(sensor.magCount);
  sensor.mx = (float)sensor.magCount[0] * sensor.factoryMagCalibration[0] - sensor.magBias[0];
  sensor.my = (float)sensor.magCount[1] * sensor.factoryMagCalibration[1] - sensor.magBias[1];
  sensor.mz = (float)sensor.magCount[2] * sensor.factoryMagCalibration[2] - sensor.magBias[2];

  sensor.updateTime();
  MadgwickQuaternionUpdate(sensor.ax, sensor.ay, sensor.az, sensor.gx * DEG_TO_RAD,
                      sensor.gy * DEG_TO_RAD, sensor.gz * DEG_TO_RAD, sensor.my,
                      sensor.mx, sensor.mz, sensor.deltat);

  sensor.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                * *(getQ()+3));
  sensor.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                * *(getQ()+2)));
  sensor.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
  sensor.pitch *= RAD_TO_DEG;
  sensor.roll *= RAD_TO_DEG;
  sensor.yaw   *= RAD_TO_DEG;

  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  sensor.yaw  -= 8.5;
}
