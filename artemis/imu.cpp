#include "imu.h"

#include <MPU9250.h>
#include <quaternionFilters.h>
#include "main.h"

void initIMU() {
  imu.initMPU9250();
  imu.initAK8963(imu.factoryMagCalibration);

  imu.getAres();
  imu.getGres();
  imu.getMres();
  // calibrate with imu.magCalMPU9250()
}

void readIMU() {
  imu.readAccelData(imu.accelCount);
  imu.ax = (float)imu.accelCount[0] * imu.aRes; // - imu.accelBias[0];
  imu.ay = (float)imu.accelCount[1] * imu.aRes; // - imu.accelBias[1];
  imu.az = (float)imu.accelCount[2] * imu.aRes; // - imu.accelBias[2];

  imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc code
  imu.gx = (float)imu.gyroCount[0] * imu.gRes;
  imu.gy = (float)imu.gyroCount[1] * imu.gRes;
  imu.gz = (float)imu.gyroCount[2] * imu.gRes;

  imu.readMagData(imu.magCount);
  imu.mx = (float)imu.magCount[0] * imu.factoryMagCalibration[0] - imu.magBias[0];
  imu.my = (float)imu.magCount[1] * imu.factoryMagCalibration[1] - imu.magBias[1];
  imu.mz = (float)imu.magCount[2] * imu.factoryMagCalibration[2] - imu.magBias[2];

  imu.updateTime();
  MadgwickQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD,
                      imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD, imu.my,
                      imu.mx, imu.mz, imu.deltat);

  imu.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                * *(getQ()+3));
  imu.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                * *(getQ()+2)));
  imu.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
  imu.pitch *= RAD_TO_DEG;
  imu.roll *= RAD_TO_DEG;
  imu.yaw   *= RAD_TO_DEG;

  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  imu.yaw  -= 8.5;
}
