#ifndef ALTIUDE_ESTIMATOR_H
#define ALTIUDE_ESTIMATOR_H
#include "stm32f4xx.h"
#include "structconfig.h"
typedef FLOAT_XYZ point_t;
typedef FLOAT_XYZ velocity_t;
typedef FLOAT_XYZ acc_t;
	
typedef struct
{
	FLOAT_ANGLE attitude;
	point_t position;
	velocity_t velocity;
	acc_t acc;
} state_t;


void Altitude_UpdateVelocity(float accWZ, float dt);
void Altitude_Estimate(state_t* estimate, const FBMTYPE* sensorData, float dt);
#endif
