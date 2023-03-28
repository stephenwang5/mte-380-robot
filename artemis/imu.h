#ifndef IMU_H
#define IMU_H

typedef enum {
  IMU_FACE_UP,
  IMU_FACE_DOWN,
  UNKNOWN,
} Orientation;

extern Orientation orientation;
extern bool launch_detected;

constexpr float magXScale = 1.23;
constexpr float magYScale = 1.23;
constexpr float magZScale = 1.19;

constexpr float magXBias = 37;
constexpr float magYBias = -450;
constexpr float magZBias = 900;

constexpr float freeFallThreshold = 0.3;

void initIMU();
void readIMU();
void imuReadLoop();
float findHeading();
void findOrientation();
float imuMagnitude();
void DetectLaunch();

#endif // IMU_H
