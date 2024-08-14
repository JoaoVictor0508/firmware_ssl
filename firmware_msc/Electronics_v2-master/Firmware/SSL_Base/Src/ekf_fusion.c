/*
 * ekf_fusion.c
 *
 *  Created on: Jun 17, 2024
 *      Author: joaov
 */

#include "ekf_fusion.h"

static void initEKF();
static void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF);
static void ekfMeasFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
static void ekfMeasJacobianFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);

static uint8_t isVisionSampleValid(const float* pVisPos, const float* pInsPos, int32_t tDelayUs);

FusionEKF fusionEKF;

void FusionEKFUpdate(const RobotSensors* pSensors, RobotState* pState){

	StateEKF stateNow;

	stateNow.accGyr[0] = pSensors->acc.linAcc[0];
	stateNow.accGyr[1] = pSensors->acc.linAcc[1];
	stateNow.accGyr[2] = pSensors->gyr.rotVel[2];

	if(pSensors->vision.updated && isVisionSampleValid(pSensors->vision.pos, fusionEKF.vision.lastPos, pSensors->vision.delay)) // see if vision is available AND if the vision sample is valid
	{
		memcpy(fusionEKF.ekf.z.pData, pSensors->vision.pos, sizeof(float)*3); // transfer the data from the vision to the Z matrix
		EKFUpdate(&fusionEKF.ekf); // updates state
	}
	else
	{
		memcpy(fusionEKF.ekf.u.pData, stateNow.accGyr, sizeof(float)*3); // transfer the data from the control to the control matrix
		EKFPredict(&fusionEKF.ekf); // predict state
	}

	pState->pos[0] = fusionEKF.ekf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.ekf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.ekf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.ekf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.ekf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.accGyr[2]; // angular velocity
}

static uint8_t isVisionSampleValid(const float* pVisPos, const float* pInsPos, int32_t tDelayUs)
{
	// validate sample, outlier rejection
	uint32_t measTime = SysTimeUSec()-tDelayUs;
	float visionDt = (measTime-fusionEKF.vision.timeLastValidSample)*1e-6f;

	float searchRadiusXY = 100.0f*fusionEKF.pConfig->visNoiseXY + fusionEKF.pConfig->outlierMaxVelXY*visionDt;
	float searchRadiusW = 2.0f*fusionEKF.pConfig->visNoiseW + fusionEKF.pConfig->outlierMaxVelW*visionDt;

	float diffX = pVisPos[0] - pInsPos[0];
	float diffY = pVisPos[1] - pInsPos[1];
	float diffXY = sqrtf(diffX*diffX+diffY*diffY);
	float diffW = AngleNormalize(pVisPos[2] - AngleNormalize(pInsPos[2]));

	if(diffXY > searchRadiusXY || fabsf(diffW) > searchRadiusW)
	{
		// this is an invalid sample, vel is impossible
		return 0;
	}

	fusionEKF.vision.timeLastValidSample = measTime;

	return 1;
}

//void FusionEKFInit(FusionEKFConfig* pConfigEkf, ModelEncConfig* pConfigModel)
//{
//	fusionEKF.pConfig = pConfigEkf;
//	ModelEncInit(&fusionEKF.modelEnc, pConfigModel, CTRL_DELTA_T);
//	initEKF();
//
//	LagElementPT1Init(&fusionEKF.lagAccel[0], 1.0f, fusionEKF.pConfig->emaAccelT, CTRL_DELTA_T);
//	LagElementPT1Init(&fusionEKF.lagAccel[1], 1.0f, fusionEKF.pConfig->emaAccelT, CTRL_DELTA_T);
//
//	float ballInitPos[2] = { 0 };
//	float ballMeasEror[] = { fusionEKF.pConfig->irMeasNoiseX, fusionEKF.pConfig->irMeasNoiseY };
//	TrackingFilter2DInit(&fusionEKF.ballFilter, CTRL_DELTA_T, ballInitPos, 1e-3f, fusionEKF.pConfig->irPosNoise, ballMeasEror);
//}

static void initEKF()
{
	EKFInit(&fusionEKF.ekf, 5, 3, 3, fusionEKF.ekfData);
	arm_mat_scale_f32(&fusionEKF.ekf.Sigma, 0.001f, &fusionEKF.ekf.Sigma);
	fusionEKF.ekf.pState = &ekfStateFunc;
	fusionEKF.ekf.pStateJacobian = &ekfStateJacobianFunc;
	fusionEKF.ekf.pMeas = &ekfMeasFunc;
	fusionEKF.ekf.pMeasJacobian = &ekfMeasJacobianFunc;

	arm_mat_identity_f32(&fusionEKF.ekf.Ex);
	arm_mat_identity_f32(&fusionEKF.ekf.Ez);

	loadNoiseCovariancesFromConfig();
}

void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU)
{
	const float dt = 0.001f;

	float gyr_w = MAT_ELEMENT(*pU, 0, 0);
	float acc_x = MAT_ELEMENT(*pU, 1, 0);
	float acc_y = MAT_ELEMENT(*pU, 2, 0);

	float p_x = MAT_ELEMENT(*pX, 0, 0);
	float p_y = MAT_ELEMENT(*pX, 1, 0);
	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

	float v_w = gyr_w;
	float a = -M_PI_2 + p_w + dt*v_w*0.5f;
	float a_x = acc_x;
	float a_y = acc_y;
//	float a_x = acc_x + v_y*v_w; // Tigers method with centrifugal force
//	float a_y = acc_y - v_x*v_w; // Tigers method with centrifugal force

//	float px1 = p_x + (arm_cos_f32(a)*v_x-arm_sin_f32(a)*v_y)*dt + (arm_cos_f32(a)*a_x-arm_sin_f32(a)*a_y)*0.5f*dt*dt;
//	float py1 = p_y + (arm_sin_f32(a)*v_x+arm_cos_f32(a)*v_y)*dt + (arm_sin_f32(a)*a_x+arm_cos_f32(a)*a_y)*0.5f*dt*dt;
	float px1 = p_x + v_x*dt + a_x*0.5f*dt*dt;
	float py1 = p_y + v_y*dt + a_y*0.5f*dt*dt;
	float vx1 = v_x + a_x*dt;
	float vy1 = v_y + a_y*dt;
	float pw1 = p_w + v_w*dt;

//	pX->pData[0][0] = px1;
//	*pX->pData[1][0] = py1;
//	*pX->pData[2][0] = pw1;
//	*pX->pData[3][0] = vx1;
//	*pX->pData[4][0] = vy1;
	MAT_ELEMENT(*pX, 0, 0) = px1; //MAT_ELEMENT ONLY WORKING BECAUSE OF CALIBRATE_IMU INCLUDE (?)
	MAT_ELEMENT(*pX, 1, 0) = py1;
	MAT_ELEMENT(*pX, 2, 0) = pw1;
	MAT_ELEMENT(*pX, 3, 0) = vx1;
	MAT_ELEMENT(*pX, 4, 0) = vy1;
}

static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF)
{
	const float dt = 0.001f;
//	float gyr_w = MAT_ELEMENT(*pU, 0, 0);
//	float acc_x = MAT_ELEMENT(*pU, 1, 0);
//	float acc_y = MAT_ELEMENT(*pU, 2, 0);
//
//	float p_w = MAT_ELEMENT(*pX, 2, 0);
//	float v_x = MAT_ELEMENT(*pX, 3, 0);
//	float v_y = MAT_ELEMENT(*pX, 4, 0);
//
//    float SF1 = gyr_w/2000.0f + p_w - M_PI_2;
//	float SF2 = arm_cos_f32(SF1)/1000.0f + (gyr_w * arm_sin_f32(SF1))/2000000.0f;
//	float SF3 = (gyr_w * arm_cos_f32(SF1))/2000000.0f;
//	float SF4 = arm_sin_f32(SF1);
//	float SF5 = acc_y - gyr_w * v_x;
//	float SF6 = arm_cos_f32(SF1);
//	float SF7 = gyr_w/1000.0f;

	arm_mat_identity_f32(pF);

//	MAT_ELEMENT(*pF, 0, 2) = - (SF5*SF6)/2000000.0f - (v_x*SF4)/1000.0f - (v_y*SF6)/1000.0f - (SF4*(acc_x + gyr_w*v_y))/2000000.0f;
//	MAT_ELEMENT(*pF, 0, 3) = SF2;
//	MAT_ELEMENT(*pF, 0, 4) = SF3 - SF4/1000.0f;
//
//	MAT_ELEMENT(*pF, 1, 2) = (v_x*SF6)/1000.0f - (SF4*SF5)/2000000.0f - (v_y*SF4)/1000.0f + (SF6*(acc_x + gyr_w*v_y))/2000000.0f;
//	MAT_ELEMENT(*pF, 1, 3) = SF4/1000.0f - SF3;
//	MAT_ELEMENT(*pF, 1, 4) = SF2;
//
//	MAT_ELEMENT(*pF, 3, 4) = SF7;
//
//	MAT_ELEMENT(*pF, 4, 3) = -SF7;

	MAT_ELEMENT(*pX, 0, 3) = dt;

	MAT_ELEMENT(*pX, 1, 4) = dt;
}

static void ekfMeasFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY)
{
	memcpy(pY->pData, pX->pData, sizeof(float)*3);
}

static void ekfMeasJacobianFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH)
{
	(void)pX;

	arm_mat_zero_f32(pH);
	MAT_ELEMENT(*pH, 0, 0) = 1.0f;
	MAT_ELEMENT(*pH, 1, 1) = 1.0f;
	MAT_ELEMENT(*pH, 2, 2) = 1.0f;
}
