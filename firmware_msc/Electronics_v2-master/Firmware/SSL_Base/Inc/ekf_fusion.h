/*
 * ekf_fusion.h
 *
 *  Created on: Jun 17, 2024
 *      Author: joaov
 */

#ifndef INC_EKF_FUSION_H_
#define INC_EKF_FUSION_H_

#include "ekf.h"
#include "ssl_main.h"
#include "lag_element.h"

#define MAT_ELEMENT(mat,r,c) ((mat).pData[(mat).numCols*(r)+(c)])

typedef struct __attribute__ ((packed)) _FusionEKFConfig
{
	float posNoiseXY;
	float posNoiseW;
	float velNoiseXY;

	float visNoiseXY;
	float visNoiseW;

	float outlierMaxVelXY;
	float outlierMaxVelW;

	float trackingCoeff;

	uint8_t visCaptureDelay;
	uint8_t fusionHorizon;

	uint16_t visionTimeoutMs;

	float emaAccelT;

} FusionEKFConfig;

typedef struct _FusionEKFTimeSlot
{
	struct _meas
	{
		float gyrAcc[3];
		float pos[3];
		uint8_t posUpdated;
	} meas;

	float insState[5];
} FusionEKFTimeSlot;

typedef struct _StateEKF
{
	float insState[5];
	float accGyr[3];
	float pos[3];
} StateEKF;

#define FUSION_EKF_MAX_DELAY 128 // must be power of 2

typedef struct _FusionEKF
{
	// EKF state vector: [ px py pw vx vy ]
	// input vector: [ gyr_w acc_x acc_y ]
	// measurement vector: [ px py pw ]
	KF kf;
	float ekfData[KF_DATA_SIZE(5, 3, 3, 5)];

	float encGyrPos[3];

	struct
	{
		uint16_t online;
		uint32_t timeLastValidSample;
		int32_t turns;
		float lastOrient;
		uint32_t numLateMeasurements;
		float lastPos[3];
	} vision;

	FusionEKFConfig* pConfig;

//	FusionEKFTimeSlot timeSlots[FUSION_EKF_MAX_DELAY];
	FusionEKFTimeSlot lastState;
	uint32_t timeSlotNow;

	bool first_vision_meas;

//	ModelEnc modelEnc;

	LagElementPT1 lagAccel[2];

	uint32_t lastTime;

} FusionEKF;

extern FusionEKF fusionEKF;
//extern ConfigFileDesc fusionEKFConfigDesc;

//void FusionEKFInit(FusionEKFConfig* pConfigEkf, ModelEncConfig* pConfigModel);
void FusionEKFInit();
void FusionEKFUpdate(const RobotSensors* pSensors, RobotState* pState);
void FusionEKFSetState(const float* pPos); // falta adicionar a velocidade local dos encoders
void FusionEKFConfigUpdateCallback(uint16_t cfgId);

float AngleNormalize(float a);
static float mod(float x, float y);

#endif /* INC_EKF_FUSION_H_ */
