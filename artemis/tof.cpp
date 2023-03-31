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
  // tof.setIntegrationTime(49);
  tof.setRangingFrequency(5);

  tof.startRanging();
}

void readToF() {
  while (1) {
    getTof();
    rtos::ThisThread::sleep_for(70ms);
  }
}

void getTof() {
  bool ready = false;
  do {
    i2cLock.lock();
    ready = tof.isDataReady();
    i2cLock.unlock();
    rtos::ThisThread::sleep_for(10ms);
  } while (!ready);

  i2cLock.lock();
  tofDataLock.lock();
  if (tof.getRangingData(&tofData)) {
    extractToF();
  } else {
    Serial.println("yikes tof fetch failed");
  }
  tofDataLock.unlock();
  i2cLock.unlock();
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

void preprocess(int16_t* m, uint16_t* s, uint8_t* r, uint8_t len) {
  // m==mean s==sigma r==reflectivity
  const uint16_t sigmaThreshold = 30;

  uint8_t bottomRow = orientation==IMU_FACE_UP ? 0 : 7;
  for (uint8_t i = 0; i < len; i++) {
    // m[i] = s[i] > sigmaThreshold ? m[i] + s[i]*2 : m[i];
    // m[i] = m[i] > 1500 ? 1500 : m[i];
    if (i >= bottomRow*8 && i < (bottomRow+1)*8) {
      m[i] = r[i] < 10 ? 200 : m[i];
    }
  }
}

int extractToF() {
  // remember that when the imu faces up,
  // 0th measurement is the bottom right, the measurements then go left and up
  // the kernel is made so that the reading order is natural
  constexpr float kernel[kernelRows][kernelCols] = {
    // top left          top right
    {0.0, 0.0, 0.0, 0.0, 0.0, },
    {0.0, 0.0, 0.0, 0.0, 0.0, },
    {0.0, 0.0, 0.0, 0.0, 0.0, },
    {2.0, -1.0, -4.0, -1.0, 2.0, },
    {1.9, -0.95, -3.8, -0.95, 1.9, },
    {1.805, -0.9025, -3.61, -0.9025, 1.805, },
    {1.71475, -0.857375, -3.4295, -0.857375, 1.71475, },
    {1.629 , -0.8145, -2.4435 , -0.8145,  1.629, },
    // bottom left                        bottom right
  };
  constexpr float threshold = 0;

  preprocess(tofData.distance_mm, tofData.range_sigma_mm, tofData.reflectance, 64);
  normalizeBuf<int16_t>(tofData.distance_mm, tofNormalized, 64);

  setZero<float>(tofDotProduct, strideLen);
  for (uint8_t stride = 0; stride < strideLen; stride++) {
    for (uint8_t row = 0; row < kernelRows; row++) {
      for (uint8_t col = 0; col < kernelCols; col++) {

        if (orientation == IMU_FACE_UP) {
          tofDotProduct[stride] += kernel[row][col] *
            tofNormalized[(kernelRows-1-row)*8 + (kernelCols-1-col) + (strideLen-1-stride)];
        } else {
          tofDotProduct[stride] += kernel[row][col] * tofNormalized[row*8 + col + stride];
        }

      }
    }
  }

  int bestMatchIdx = 0;
  float bestMatch = tofDotProduct[0];
  for (int i = 1; i < 4; i++) {
    if (tofDotProduct[i] > bestMatch) {
      bestMatchIdx = i;
      bestMatch = tofDotProduct[i];
    }
  }
  tofMatch = bestMatchIdx;

  if (bufMax<float>(tofDotProduct, 4) < threshold) {
    tofMatch = -1;
  }

  if (bufMax<int16_t>(tofData.distance_mm, 8) < 200) {
    tofMatch = -1;
  }

  return bestMatchIdx;
}
