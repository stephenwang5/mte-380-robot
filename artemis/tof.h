#ifndef TOF_H
#define TOF_H

extern float tofDotProduct[5];
extern float tofNormalized[64];
extern int tofMatch;

void initToF();
void readToF();
int extractToF();

#endif // TOF_H
