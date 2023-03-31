#ifndef TOF_H
#define TOF_H

constexpr uint8_t kernelRows = 8;
constexpr uint8_t kernelCols = 5;
constexpr uint8_t strideLen = kernelRows - kernelCols + 1;

extern float tofDotProduct[5];
extern float tofNormalized[64];
extern int tofMatch;

void initToF();
void readToF();
void getTof();
int extractToF();
int16_t minDistance(uint8_t row);
int16_t avgDistance(uint8_t row);

#endif // TOF_H
