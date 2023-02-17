#ifndef TOF_H
#define TOF_H

#include <SparkFun_VL53L5CX_Library.h>

VL53L5CX_ResultsData ToF_data;

void ToF_begin(SparkFun_VL53L5CX& sensor) {
  if (!sensor.begin()) {
    Serial.println("tough luck. tof not found");
    while (1);
  }

#ifdef DEBUG
  Serial.println("upload complete!");
#endif

  sensor.setResolution(64);

  sensor.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  sensor.setRangingFrequency(15);

  sensor.startRanging();
}

void ToF_read(SparkFun_VL53L5CX& sensor) {
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&ToF_data)) {

#ifdef DEBUG
      Serial.print(millis() - start_time);
      Serial.print(" ms ");
      start_time = millis();
      Serial.println(" new data available");
#endif

    } else {
      Serial.println("yikes tof fetch failed");
    }
  }
}

#endif // TOF_H
