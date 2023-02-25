#include "tof.h"

ToF::ToF() : sensor(), data() {}

void ToF::begin() {
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

void ToF::read() {
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&data)) {

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

void ToF::filter() {
  constexpr float kernel[8][5] = {
    {0, 0, 0, 0, 0}, // row 0
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
  };

  float dotProduct[4] = {0};
  for (int stride = 0; stride < 4; stride++) {
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 5; col++) {
        dotProduct[stride] += kernel[row][col+stride] * data.distance_mm[row*8 + col];
      }
    }
  }

  int bestMatchIdx;
  float bestMatch = max(dotProduct[0], dotProduct[1]);
  for (int i = 1; i < 4; i++) {
    float tempMax = max(dotProduct[i], dotProduct[i+1]);
    if (tempMax > bestMatch) {
      bestMatchIdx = i;
      bestMatch = tempMax;
    }
  }
  filterResult = bestMatchIdx;

}
