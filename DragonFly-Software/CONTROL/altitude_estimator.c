#include "altitude_estimator.h"
#include "fbm320.h"
#include "structconfig.h"
#include "math.h"
#define G 9.81f;

//typedef FLOAT_XYZ point_t;
//typedef FLOAT_XYZ velocity_t;
//typedef FLOAT_XYZ acc_t;
//	
//typedef struct
//{
//	FLOAT_ANGLE attitude;
//	point_t position;
//	velocity_t velocity;
//	acc_t acc;
//} state_t;
typedef struct 
{
  float estimatedZ; // The current Z estimate, has same offset as asl
  float velocityZ;  // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float estAlphaAsl;
  float velocityFactor;
  float vAccDeadband; // 垂直加速度死区
  float velZAlpha;    // Blending factor to avoid vertical speed to accumulate error
  float estimatedVZ;
}selfState_t;

 static selfState_t selfState = 
{
  .estimatedZ = 0.0f,
  .velocityZ = 0.0f,
  .estAlphaAsl = 0.997f,
  .velocityFactor = 1.0f,
  .vAccDeadband = 0.04f,
  .velZAlpha = 0.995f,
  .estimatedVZ = 0.0f,
};
float deadband(float value, const float threshold)
{
	if (fabs(value) < threshold)
	{
		value = 0;
	}
	else if (value > 0)
	{
		value -= threshold;
	}
	else if (value < 0)
	{
		value += threshold;
	}
	return value;
}

static void AltitudeEstimateInternal(state_t* estimate, const FBMTYPE* sensorData, float dt, selfState_t* selfState);
static void AltitudeUpdateVelocityInternal(float accWZ, float dt, selfState_t* selfState);

void Altitude_Estimate(state_t* estimate, const FBMTYPE* sensorData, float dt) 
{
  AltitudeEstimateInternal(estimate, sensorData, dt, &selfState);
}

void Altitude_UpdateVelocity(float accWZ, float dt) 
{
  AltitudeUpdateVelocityInternal(accWZ, dt, &selfState);
}

static void AltitudeEstimateInternal(state_t* estimate, const FBMTYPE* sensorData, float dt, selfState_t* selfState)
{
  float filteredZ;
//  static float prev_estimatedZ = 0;
    // FIXME: A bit of an hack to init IIR filter
    if (selfState->estimatedZ == 0.0f)
		{
      filteredZ = sensorData->Altitude; 
    } 
		else 
		{
      // IIR filter asl //高度融合
      filteredZ = selfState->estAlphaAsl * selfState->estimatedZ + (1.0f - selfState->estAlphaAsl) * sensorData->Altitude;
			//printf("sensorData->Altitude:%0.2f\r\n",sensorData->Altitude);
    }
    // Use asl as base and add velocity changes.
    selfState->estimatedZ = filteredZ + (selfState->velocityFactor * selfState->velocityZ * dt);


  estimate->position.X = 0.0f;
  estimate->position.Y = 0.0f;
  estimate->position.Z = selfState->estimatedZ;
//  estimate->velocity.Z = (selfState->estimatedZ - prev_estimatedZ) / dt;
//  selfState->estimatedVZ = estimate->velocity.Z;
//  prev_estimatedZ = selfState->estimatedZ;
	//printf("estimated Altitude:%0.2f\n", filteredZ);
	//printf("state->estimatedZ:%0.2f\r\n",selfState->estimatedZ);
}

static void AltitudeUpdateVelocityInternal(float accWZ, float dt, selfState_t* selfState) 
{
  selfState->velocityZ += deadband(accWZ, selfState->vAccDeadband) * dt;// * G;
  selfState->velocityZ *= selfState->velZAlpha;
	//printf("selfState->velocityZ:%0.2f\r\n",selfState->velocityZ);
}
