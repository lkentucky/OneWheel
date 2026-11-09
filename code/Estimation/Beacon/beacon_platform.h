#ifndef BEACON_PLATFORM_H
#define BEACON_PLATFORM_H
#include "beacon_types.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#ifndef PI
#define PI 3.14159265358979323846f
#endif
#ifndef DEG2RAD
#define DEG2RAD (PI/180.0f)
#define RAD2DEG (180.0f/PI)
#endif
extern Mono_beacon mono_beacon;
#include "image_process.h"
#include "component.h"
#include "contourfilter.h"
#include "findcontours.h"
#include "target_classifier.h"
#include "tragectory_planning.h"
#include "pixel2world.h"
#include "tophat.h"
#include "morphology.h"
#include "mlp.h"
#include "ocsvm.h"
#include "otus.h"
#include "sobel.h"
#endif
