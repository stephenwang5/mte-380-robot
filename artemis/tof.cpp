#include "tof.h"

#include "main.h"

float tofDotProduct[5] = {0};
float tofNormalized[64] = {0};
int tofMatch = -1;

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
    while (!tof.isDataReady()) {
      rtos::ThisThread::sleep_for(10ms);
    }
    i2cLock.lock();
    if (tof.getRangingData(&tofData)) {
      extractToF();
    } else {
      Serial.println("yikes tof fetch failed");
    }
    i2cLock.unlock();

    rtos::ThisThread::sleep_for(70ms);
  }
}

template<typename T>
float bufMax(T* buf, uint8_t len) {
  float temp = buf[0];
  for (uint8_t i = 1; i < len; i++) {
    if (buf[i] > temp) temp = buf[i];
  }
  return temp;
}
float bufMax(float* buf, uint8_t len) {
  float temp = buf[0];
  for (uint8_t i = 1; i < len; i++) {
    if (buf[i] > temp) temp = buf[i];
  }
  return temp;
}

template<typename T>
void normalizeBuf(T* src, float* dest, uint8_t len) {
  float max = bufMax<T>(src, len);
  for (uint8_t i = 0; i < len; i++) {
    dest[i] = src[i] / max;
  }
}

template<typename T>
void minBuf(T* src, float* dest, int arg, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    dest[i] = src[i] < arg ? src[i] : arg;
  }
}

template <typename T>
void setZero(T* buf, uint8_t len) {
  for (int i = 0; i < len; i++) {
    buf[i] = 0;
  }
}

void preprocess(int16_t* m, uint16_t* s, uint8_t len) {
  const uint16_t sigmaThreshold = 10;
  for (uint8_t i = 0; i < len; i++) {
    m[i] = s[i] > sigmaThreshold ? 1000 : m[i];
    m[i] = m[i] > 1000 ? 1000 : m[i];
  }
}

int extractToF() {
  constexpr float kernel[kernelRows][kernelCols] = {
    {2.0, -1.0, -4.0, -1.0, 2.0, },
    {1.9, -0.95, -3.8, -0.95, 1.9, },
    {1.805, -0.9025, -3.61, -0.9025, 1.805, },
    {1.71475, -0.857375, -3.4295, -0.857375, 1.71475, },
    {2.0, -1.0, -3.0, -1.0, 2.0, },
    {0.0, 0.0, 0.0, 0.0, 0.0, },
    {0.0, 0.0, 0.0, 0.0, 0.0, },
    {0.0, 0.0, 0.0, 0.0, 0.0, },
  };
  constexpr float threshold = -2;

  tofDataLock.lock();

  // minBuf<int16_t>(tofData.distance_mm, tofNormalized, 1000, 64);
  preprocess(tofData.distance_mm, tofData.range_sigma_mm, 64);
  normalizeBuf<float>(tofNormalized, tofNormalized, 64);

  setZero<float>(tofDotProduct, strideLen);
  for (int stride = 0; stride < strideLen; stride++) {
    for (int row = 0; row < kernelRows; row++) {
      for (int col = 0; col < kernelCols; col++) {
        tofDotProduct[stride] += kernel[row][col] * tofNormalized[row*8 + col + stride];
      }
    }
  }

  int bestMatchIdx;
  float bestMatch = max(tofDotProduct[0], tofDotProduct[1]);
  for (int i = 2; i < 4; i++) {
    if (tofDotProduct[i] > bestMatch) {
      bestMatchIdx = i;
      bestMatch = tofDotProduct[i];
    }
  }
  tofMatch = bestMatchIdx;

  if (bufMax<float>(tofDotProduct, 4) < threshold) {
    tofMatch = -1;
  }

  tofDataLock.unlock();

  return bestMatchIdx;
}
