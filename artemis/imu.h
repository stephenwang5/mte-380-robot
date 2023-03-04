#ifndef IMU_H
#define IMU_H

typedef enum {
  IMU_FACE_UP,
  IMU_FACE_DOWN,
} Orientation;

extern Orientation orientation;

void initIMU();
void readIMU();
void imuReadLoop();
void findOrientation();

#endif // IMU_H
