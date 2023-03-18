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
int extractToF();

#endif // TOF_H
