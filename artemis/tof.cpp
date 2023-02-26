#include "tof.h"

#include "main.h"

void initToF() {
  if (!tof.begin(0x52 >> 1, i2c)) {
    Serial.println("tough luck. tof not found");
    while (1);
  }

#ifdef DEBUG
  Serial.println("upload complete!");
#endif

  tof.setResolution(64);

  tof.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  tof.setRangingFrequency(15);

  tof.startRanging();
}

void readToF() {
  while (1) {
    i2cLock.lock();
    if (tof.isDataReady()) {
      if (tof.getRangingData(&tofData)) {
      } else {
        Serial.println("yikes tof fetch failed");
      }
    }
    i2cLock.unlock();

    rtos::ThisThread::sleep_for(100ms);
  }
}

int extractToF() {
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
        dotProduct[stride] += kernel[row][col+stride] * tofData.distance_mm[row*8 + col];
      }
    }
  }

  int bestMatchIdx;
  float bestMatch = max(dotProduct[0], dotProduct[1]);
  for (int i = 2; i < 4; i++) {
    if (dotProduct[i] > bestMatch) {
      bestMatchIdx = i;
      bestMatch = dotProduct[i];
    }
  }

  return bestMatchIdx;
}
