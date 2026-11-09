#ifndef OCSVM_H
#define OCSVM_H

#include "beacon_platform.h"

#define NUM_SV 498
#define feature_num 4

void set_features(float x,float y, float s, float a,float g);
int get_prediction(void);

int OCSVM_Filter_test(ContourInfo* contourinfo);

#endif
