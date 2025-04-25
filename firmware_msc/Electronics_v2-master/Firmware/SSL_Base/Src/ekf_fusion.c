/*
 * ekf_fusion.c
 *
 *  Created on: Jun 17, 2024
 *      Author: joaov
 */

#include "ekf_fusion.h"

static void initEKF();
static void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, float dt);
static void ekfStateFunc_encoder(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, float dt);
static void ekfStateFunc_model(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, float dt);

static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF, float dt);
static void ekfStateJacobianFunc_model(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF, float dt);
static void ekfStateJacobianFunc_encoder(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF, float dt);

static void ekfMeasFuncPoseState(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
static void ekfMeasFuncFullState(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);

static void ekfMeasJacobianFuncPoseState(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);
static void ekfMeasJacobianFuncFullState(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);

static float visionMultiTurnCorrection(float visionOrientation);

void Vector2fTurnLocal2Global(float globalOrientation, float localX, float localY, float* pGlobalX, float* pGlobalY);

static uint8_t isVisionSampleValid(const float* pVisPos, const float* pInsPos);
static void loadNoiseCovariancesFromConfig();

FusionEKF fusionEKF;

uint8_t arm_mat_is_nan_f32(const arm_matrix_instance_f32* A)
{
	for(uint32_t i = 0; i < A->numCols*A->numRows; i++)
	{
		if(isnanf(A->pData[i]))
			return 1;
	}

	return 0;
}

void FusionEKFUpdate(const RobotSensors* pSensors, RobotState* pState){

	StateEKF stateNow;

	uint32_t new_sample_time = HAL_GetTick();

	float dt = (new_sample_time-fusionEKF.lastTime)*1e-3f;

	stateNow.accGyr[0] = pSensors->acc.linAcc[0]; // m/s^2
	stateNow.accGyr[1] = pSensors->acc.linAcc[1]; // m/s^2
	stateNow.accGyr[2] = pSensors->gyr.rotVel[2]; // convert º/s to rad/s

	if(fusionEKF.first_vision_meas == true)
	{
		FusionEKFSetState(pSensors->vision.pos);
		memcpy(fusionEKF.vision.lastPos, pSensors->vision.pos, sizeof(float)*3);
		fusionEKF.first_vision_meas = false;
	}

	memcpy(fusionEKF.kf.u.pData, stateNow.accGyr, sizeof(float)*3); // transfer the data from the control to the control matrix
	KFPredict(&fusionEKF.kf, dt); // predict state

	if(pSensors->vision.updated)// || (new_sample_time - fusionEKF.lastUpdateTime > 1000))
	{
		memcpy(fusionEKF.kf.z.pData, pSensors->vision.pos, sizeof(float)*3); // transfer the data from the vision to the Z matrix
		KFUpdate(&fusionEKF.kf); // update state

		fusionEKF.lastUpdateTime = new_sample_time;
	}

	if(arm_mat_is_nan_f32(&fusionEKF.kf.x))
	{
		initEKF();
		fusionEKF.vision.online = 0;
	}

	fusionEKF.lastTime = new_sample_time;

	pState->pos[0] = fusionEKF.kf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.kf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.kf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.kf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.kf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.accGyr[2]; // angular velocity
}

void FusionEKFUpdate_model_vision(const RobotSensors* pSensors, RobotState* pState){

	StateEKF stateNow;

	uint32_t new_sample_time = HAL_GetTick();

	float dt = (new_sample_time-fusionEKF.lastTime)*1e-3f;

	stateNow.vel[0] = pSensors->theoreticalVel.theoVel[0]; // m/s
	stateNow.vel[1] = pSensors->theoreticalVel.theoVel[1]; // m/s
	stateNow.vel[2] = pSensors->theoreticalVel.theoVel[2]; // rad/s

	if(fusionEKF.first_vision_meas == true)
	{
		FusionEKFSetState(pSensors->vision.pos);
		memcpy(fusionEKF.vision.lastPos, pSensors->vision.pos, sizeof(float)*3);
		fusionEKF.first_vision_meas = false;
	}

	memcpy(fusionEKF.kf.u.pData, stateNow.vel, sizeof(float)*3); // transfer the data from the control to the control matrix
	KFPredict(&fusionEKF.kf, dt); // predict state

	if(pSensors->vision.updated) // see if vision is available AND if the vision sample is valid
	{
		memcpy(fusionEKF.kf.z.pData, pSensors->vision.pos, sizeof(float)*3); // transfer the data from the vision to the Z matrix
		KFUpdate(&fusionEKF.kf); // update state

		if(!fusionEKF.vision.online)
		{
			fusionEKF.vision.online = 1;
		}
	}

	if(arm_mat_is_nan_f32(&fusionEKF.kf.x))
	{
		initEKF();
		fusionEKF.vision.online = 0;
	}

	fusionEKF.lastTime = new_sample_time;

	pState->pos[0] = fusionEKF.kf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.kf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.kf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.kf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.kf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.vel[2]; // angular velocity
}

void FusionEKFUpdate_model_encoder(const RobotSensors* pSensors, RobotState* pState){
	StateEKF stateNow;

	uint32_t new_sample_time = HAL_GetTick();

	float encVelGlobal[2];

	float update_pose[5];

	float dt = (new_sample_time-fusionEKF.lastTime)*1e-3f;

	if(fusionEKF.first_vision_meas == true)
	{
		FusionEKFSetState(pSensors->vision.pos);
		memcpy(fusionEKF.vision.lastPos, pSensors->vision.pos, sizeof(float)*3);
		fusionEKF.first_vision_meas = false;

		pState->pos_enc[0] = pSensors->vision.pos[0];
		pState->pos_enc[1] = pSensors->vision.pos[1];
		pState->pos_enc[2] = pSensors->vision.pos[2];
		pState->vel_enc[0] = pSensors->encoder.localVel[0];
		pState->vel_enc[1] = pSensors->encoder.localVel[1];
		pState->vel_enc[2] = pSensors->encoder.localVel[2];
	}

	Vector2fTurnLocal2Global(pSensors->vision.pos[2], pSensors->encoder.localVel[0], pSensors->encoder.localVel[1], encVelGlobal, encVelGlobal+1);

	stateNow.vel[0] = pSensors->theoreticalVel.theoVel[0]; // m/s
	stateNow.vel[1] = pSensors->theoreticalVel.theoVel[1]; // m/s
	stateNow.vel[2] = pSensors->theoreticalVel.theoVel[2]; // rad/s

	pState->pos_enc[0] = pState->pos_enc[0] + encVelGlobal[0] * dt;
	pState->pos_enc[1] = pState->pos_enc[1] + encVelGlobal[1] * dt;
	pState->pos_enc[2] = pState->pos_enc[2] + pSensors->encoder.localVel[2] * dt;

	update_pose[0] = pState->pos_enc[0]; // X position calculated using encoder in the update
	update_pose[1] = pState->pos_enc[1]; // Y position calculated using encoder in the update
	update_pose[2] = pState->pos_enc[2]; // theta position calculated using encoder in the update
	update_pose[3] = pSensors->encoder.localVel[0];
	update_pose[4] = pSensors->encoder.localVel[1];

	memcpy(fusionEKF.kf.u.pData, stateNow.vel, sizeof(float)*3); // transfer the data from the control to the control matrix
	KFPredict(&fusionEKF.kf, dt); // predict state

	memcpy(fusionEKF.kf.z.pData, update_pose, sizeof(float)*5); // transfer the data from the vision to the Z matrix
	KFUpdate(&fusionEKF.kf); // update state

	if(arm_mat_is_nan_f32(&fusionEKF.kf.x))
	{
		initEKF();
		fusionEKF.vision.online = 0;
	}

	fusionEKF.lastTime = new_sample_time;

	pState->pos[0] = fusionEKF.kf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.kf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.kf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.kf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.kf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.vel[2]; // angular velocity
}

void FusionEKFUpdate_model_imu(const RobotSensors* pSensors, RobotState* pState){
	StateEKF stateNow;

	uint32_t new_sample_time = HAL_GetTick();

	float accelGlobal[2];
	float accelGlobalVel[2];

	float update_pose[5];

	float dt = (new_sample_time-fusionEKF.lastTime)*1e-3f;

	stateNow.vel[0] = pSensors->theoreticalVel.theoVel[0]; // m/s
	stateNow.vel[1] = pSensors->theoreticalVel.theoVel[1]; // m/s
	stateNow.vel[2] = pSensors->theoreticalVel.theoVel[2]; // rad/s

	stateNow.accGyr[0] = pSensors->acc.linAcc[0]; //m/s^2
	stateNow.accGyr[1] = pSensors->acc.linAcc[1]; //m/s^2
	stateNow.accGyr[2] = pSensors->gyr.rotVel[2]; // convert º/s to rad/s

	if(fusionEKF.first_vision_meas == true)
	{
		FusionEKFSetState(pSensors->vision.pos);
		memcpy(fusionEKF.vision.lastPos, pSensors->vision.pos, sizeof(float)*3);
		fusionEKF.first_vision_meas = false;

		pState->pos_accel[0] = pSensors->vision.pos[0];
		pState->pos_accel[1] = pSensors->vision.pos[1];
		pState->pos_accel[2] = pSensors->vision.pos[2];
		pState->vel_accel[0] = pSensors->encoder.localVel[0];
		pState->vel_accel[1] = pSensors->encoder.localVel[1];
		pState->vel_accel[2] = pSensors->encoder.localVel[2];
	}

	Vector2fTurnLocal2Global(pSensors->vision.pos[2], pState->vel_accel[0], pState->vel_accel[1], accelGlobalVel, accelGlobalVel+1);
	Vector2fTurnLocal2Global(pSensors->vision.pos[2], stateNow.accGyr[0], stateNow.accGyr[1], accelGlobal, accelGlobal+1);

	pState->pos_accel[0] = pState->pos_accel[0] + accelGlobalVel[0] * dt + accelGlobal[0] * 0.5 * dt * dt;
	pState->pos_accel[1] = pState->pos_accel[1] + accelGlobalVel[1] * dt + accelGlobal[1] * 0.5 * dt * dt;
	pState->pos_accel[2] += stateNow.accGyr[2] * dt;
	pState->vel_accel[0] += stateNow.accGyr[0] * dt;
	pState->vel_accel[1] += stateNow.accGyr[1] * dt;

	update_pose[0] = pState->pos_accel[0];
	update_pose[1] = pState->pos_accel[1];
	update_pose[2] = pState->pos_accel[2];
	update_pose[3] = pState->vel_accel[0];
	update_pose[4] = pState->vel_accel[1];

	memcpy(fusionEKF.kf.u.pData, stateNow.vel, sizeof(float)*3); // transfer the data from the control to the control matrix
	KFPredict(&fusionEKF.kf, dt); // predict state

	memcpy(fusionEKF.kf.z.pData, update_pose, sizeof(float)*5); // transfer the data from the pose updated to the Z matrix
	KFUpdate(&fusionEKF.kf); // update state

	if(arm_mat_is_nan_f32(&fusionEKF.kf.x))
	{
		initEKF();
		fusionEKF.vision.online = 0;
	}

	fusionEKF.lastTime = new_sample_time;

	pState->imu_counter += 1;

	if(pState->imu_counter == 4){
		pState->vel_accel[0] = fusionEKF.kf.x.pData[3];
		pState->vel_accel[1] = fusionEKF.kf.x.pData[4];
	}

//	pState->vel_accel[0] = fusionEKF.kf.x.pData[3];
//	pState->vel_accel[1] = fusionEKF.kf.x.pData[4];

	pState->pos[0] = fusionEKF.kf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.kf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.kf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.kf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.kf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.vel[2]; // angular velocity
}

void FusionEKFUpdate_imu_vision(const RobotSensors* pSensors, RobotState* pState){

	StateEKF stateNow;

	uint32_t new_sample_time = HAL_GetTick();

	float dt = (new_sample_time-fusionEKF.lastTime)*1e-3f;

	if(fusionEKF.lastTime == 0){
		dt = 0;
	}

	stateNow.accGyr[0] = pSensors->acc.linAcc[0]; // m/s^2
	stateNow.accGyr[1] = pSensors->acc.linAcc[1]; // m/s^2
	stateNow.accGyr[2] = pSensors->gyr.rotVel[2];// * M_PI / 180; // convert º/s to rad/s

	if(fusionEKF.first_vision_meas == true)
	{
		FusionEKFSetState(pSensors->vision.pos);
		memcpy(fusionEKF.vision.lastPos, pSensors->vision.pos, sizeof(float)*3);
		fusionEKF.first_vision_meas = false;
	}

	memcpy(fusionEKF.kf.u.pData, stateNow.accGyr, sizeof(float)*3); // transfer the data from the control to the control matrix
	KFPredict(&fusionEKF.kf, dt); // predict state

	if(pSensors->vision.updated)
	{
		memcpy(fusionEKF.kf.z.pData, pSensors->vision.pos, sizeof(float)*3); // transfer the data from the vision to the Z matrix
		KFUpdate(&fusionEKF.kf); // update state

//		fusionEKF.predict_now = true;
	}

	if(arm_mat_is_nan_f32(&fusionEKF.kf.x))
	{
		initEKF();
		fusionEKF.vision.online = 0;
	}

	fusionEKF.lastTime = new_sample_time;

	fusionEKF.vision.online = 1;

	pState->pos[0] = fusionEKF.kf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.kf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.kf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.kf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.kf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.accGyr[2]; // angular velocity

//	for(int i = 0; i < fusionEKF.kf.Sigma.numRows; i++)
//	{
//		for(int j = 0; j < fusionEKF.kf.Sigma.numCols; j++)
//		{
//			pState->Sigma[i][j] = MAT_ELEMENT(fusionEKF.kf.Sigma, i, j);
//		}
//	}
//
//	for(int k = 0; k < fusionEKF.kf.K.numRows; k++)
//	{
//		for(int a = 0; a < fusionEKF.kf.K.numCols; a++)
//		{
//			pState->kalman_gain[k][a] = MAT_ELEMENT(fusionEKF.kf.K, k, a);
//		}
//	}
}

void FusionEKFUpdate_imu_encoder(const RobotSensors* pSensors, RobotState* pState){

	StateEKF stateNow;

	uint32_t new_sample_time = HAL_GetTick();

	float encVelGlobal[2];

	float update_pose[5];

	float dt = (new_sample_time-fusionEKF.lastTime)*1e-3f;

	stateNow.accGyr[0] = pSensors->acc.linAcc[0]; // m/s^2
	stateNow.accGyr[1] = pSensors->acc.linAcc[1]; // m/s^2
	stateNow.accGyr[2] = pSensors->gyr.rotVel[2]; // convert º/s to rad/s

	stateNow.vel[0] = pSensors->encoder.localVel[0]; // m/s
	stateNow.vel[1] = pSensors->encoder.localVel[1]; // m/s
	stateNow.vel[2] = pSensors->encoder.localVel[2]; // rad/s

	if(fusionEKF.first_vision_meas == true)
	{
		FusionEKFSetState(pSensors->vision.pos);
		memcpy(fusionEKF.vision.lastPos, pSensors->vision.pos, sizeof(float)*3);
		fusionEKF.first_vision_meas = false;

		pState->pos_enc[0] = pSensors->vision.pos[0];
		pState->pos_enc[1] = pSensors->vision.pos[1];
		pState->pos_enc[2] = pSensors->vision.pos[2];
		pState->vel_enc[0] = pSensors->encoder.localVel[0];
		pState->vel_enc[1] = pSensors->encoder.localVel[1];
		pState->vel_enc[2] = pSensors->encoder.localVel[2];
	}

//	if(pSensors->vision.noVision == true)
//	{
//		if(fusionEKF.vision.online == true)
//		{
//			fusionEKF.vision.online = false;
//			pState->pos_enc[0] = pState->pos[0];
//			pState->pos_enc[1] = pState->pos[1];
//			pState->pos_enc[2] = pState->pos[2];
//			pState->vel_enc[0] = pSensors->encoder.localVel[0];
//			pState->vel_enc[1] = pSensors->encoder.localVel[1];
//			pState->vel_enc[2] = pSensors->encoder.localVel[2];
//		}
//	}
//	else{
//		fusionEKF.vision.online = true;
//	}

	Vector2fTurnLocal2Global(pSensors->vision.pos[2], pSensors->encoder.localVel[0], pSensors->encoder.localVel[1], encVelGlobal, encVelGlobal+1);

	pState->pos_enc[0] = pState->pos_enc[0] + encVelGlobal[0] * dt;
	pState->pos_enc[1] = pState->pos_enc[1] + encVelGlobal[1] * dt;
	pState->pos_enc[2] = pState->pos_enc[2] + pSensors->encoder.localVel[2] * dt;

	update_pose[0] = pState->pos_enc[0]; // X position calculated using encoder in the update
	update_pose[1] = pState->pos_enc[1]; // Y position calculated using encoder in the update
	update_pose[2] = pState->pos_enc[2]; // theta position calculated using encoder in the update
	update_pose[3] = pSensors->encoder.localVel[0];
	update_pose[4] = pSensors->encoder.localVel[1];

	memcpy(fusionEKF.kf.u.pData, stateNow.accGyr, sizeof(float)*3); // transfer the data from the control to the control matrix
	KFPredict(&fusionEKF.kf, dt); // predict state

	memcpy(fusionEKF.kf.z.pData, update_pose, sizeof(float)*5); // transfer the data from the update pose to the Z matrix
	KFUpdate(&fusionEKF.kf); // update state

	if(arm_mat_is_nan_f32(&fusionEKF.kf.x))
	{
		initEKF();
		fusionEKF.vision.online = 0;
	}

	fusionEKF.lastTime = new_sample_time;

	pState->pos[0] = fusionEKF.kf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.kf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.kf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.kf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.kf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.accGyr[2]; // angular velocity

	pState->global_vel[0] = encVelGlobal[0];
	pState->global_vel[1] = encVelGlobal[1];
}

void FusionEKFUpdate_encoder_vision(const RobotSensors* pSensors, RobotState* pState){
	StateEKF stateNow;

	uint32_t new_sample_time = HAL_GetTick();

	float dt = (new_sample_time-fusionEKF.lastTime)*1e-3f;

	stateNow.vel[0] = pSensors->encoder.localVel[0]; // m/s
	stateNow.vel[1] = pSensors->encoder.localVel[1]; // m/s
	stateNow.vel[2] = pSensors->encoder.localVel[2]; // rad/s

	if(fusionEKF.first_vision_meas == true)
	{
		FusionEKFSetState(pSensors->vision.pos);
		memcpy(fusionEKF.vision.lastPos, pSensors->vision.pos, sizeof(float)*3);
		fusionEKF.first_vision_meas = false;
	}

	memcpy(fusionEKF.kf.u.pData, stateNow.vel, sizeof(float)*3); // transfer the data from the control to the control matrix
	KFPredict(&fusionEKF.kf, dt); // predict state

	if(pSensors->vision.updated) // see if vision is available AND if the vision sample is valid
	{
		memcpy(fusionEKF.kf.z.pData, pSensors->vision.pos, sizeof(float)*3); // transfer the data from the vision to the Z matrix
		KFUpdate(&fusionEKF.kf); // update state

		if(!fusionEKF.vision.online)
		{
			fusionEKF.vision.online = 1;
		}
	}

	if(arm_mat_is_nan_f32(&fusionEKF.kf.x))
	{
		initEKF();
		fusionEKF.vision.online = 0;
	}

	fusionEKF.lastTime = new_sample_time;

	pState->pos[0] = fusionEKF.kf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.kf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.kf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.kf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.kf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.accGyr[2]; // angular velocity

	for(int i = 0; i < fusionEKF.kf.Sigma.numRows; i++)
	{
		for(int j = 0; j < fusionEKF.kf.Sigma.numCols; j++)
		{
			pState->Sigma[i][j] = MAT_ELEMENT(fusionEKF.kf.Sigma, i, j);
		}
	}

	for(int k = 0; k < fusionEKF.kf.K.numRows; k++)
	{
		for(int a = 0; a < fusionEKF.kf.K.numCols; a++)
		{
			pState->kalman_gain[k][a] = MAT_ELEMENT(fusionEKF.kf.K, k, a);
		}
	}
}

void FusionEKFUpdate_encoder_imu(const RobotSensors* pSensors, RobotState* pState){
	StateEKF stateNow;

	uint32_t new_sample_time = HAL_GetTick();

	float accelGlobal[2];
	float accelGlobalVel[2];

	float update_pose[5];

	float dt = (new_sample_time-fusionEKF.lastTime)*1e-3f;

	if(fusionEKF.first_vision_meas == true)
	{
		FusionEKFSetState(pSensors->vision.pos);
		memcpy(fusionEKF.vision.lastPos, pSensors->vision.pos, sizeof(float)*3);
		fusionEKF.first_vision_meas = false;

		pState->pos_accel[0] = pSensors->vision.pos[0];
		pState->pos_accel[1] = pSensors->vision.pos[1];
		pState->pos_accel[2] = pSensors->vision.pos[2];
		pState->vel_accel[0] = pSensors->encoder.localVel[0];
		pState->vel_accel[1] = pSensors->encoder.localVel[1];
		pState->vel_accel[2] = pSensors->encoder.localVel[2];
	}

	if(pSensors->vision.noVision == true)
	{
		if(fusionEKF.vision.online == true)
		{
			fusionEKF.vision.online = false;
			pState->pos_accel[0] = pState->pos[0];
			pState->pos_accel[1] = pState->pos[1];
			pState->pos_accel[2] = pState->pos[2];
			pState->vel_accel[0] = pSensors->encoder.localVel[0];
			pState->vel_accel[1] = pSensors->encoder.localVel[1];
			pState->vel_accel[2] = pSensors->encoder.localVel[2];
		}
	}
	else{
		fusionEKF.vision.online = true;
	}

	stateNow.vel[0] = pSensors->encoder.localVel[0]; // m/s
	stateNow.vel[1] = pSensors->encoder.localVel[1]; // m/s
	stateNow.vel[2] = pSensors->encoder.localVel[2]; // rad/s

	stateNow.accGyr[0] = pSensors->acc.linAcc[0]; // m/s^2
	stateNow.accGyr[1] = pSensors->acc.linAcc[1]; // m/s^2
	stateNow.accGyr[2] = pSensors->gyr.rotVel[2]; // * M_PI / 180.0; // convert º/s to rad/s

	Vector2fTurnLocal2Global(pSensors->vision.pos[2], pState->vel_accel[0], pState->vel_accel[1], accelGlobalVel, accelGlobalVel+1);
	Vector2fTurnLocal2Global(pSensors->vision.pos[2], stateNow.accGyr[0], stateNow.accGyr[1], accelGlobal, accelGlobal+1);

	pState->pos_accel[0] = pState->pos_accel[0] + accelGlobalVel[0] * dt + accelGlobal[0] * 0.5 * dt * dt;
	pState->pos_accel[1] = pState->pos_accel[1] + accelGlobalVel[1] * dt + accelGlobal[1] * 0.5 * dt * dt;
	pState->pos_accel[2] += stateNow.accGyr[2] * dt;
	pState->vel_accel[0] += stateNow.accGyr[0] * dt;
	pState->vel_accel[1] += stateNow.accGyr[1] * dt;

	update_pose[0] = pState->pos_accel[0];
	update_pose[1] = pState->pos_accel[1];
	update_pose[2] = pState->pos_accel[2];
	update_pose[3] = pState->vel_accel[0];
	update_pose[4] = pState->vel_accel[1];

	memcpy(fusionEKF.kf.u.pData, stateNow.vel, sizeof(float)*3); // transfer the data from the control to the control matrix
	KFPredict(&fusionEKF.kf, dt); // predict state

	memcpy(fusionEKF.kf.z.pData, update_pose, sizeof(float)*5); // transfer the data from the pose updated to the Z matrix
	KFUpdate(&fusionEKF.kf); // update state

	if(arm_mat_is_nan_f32(&fusionEKF.kf.x))
	{
		initEKF();
		fusionEKF.vision.online = 0;
	}

	fusionEKF.lastTime = new_sample_time;

	pState->imu_counter += 1;

//	if(pState->imu_counter == 4){
//		pState->vel_accel[0] = fusionEKF.kf.x.pData[3];
//		pState->vel_accel[1] = fusionEKF.kf.x.pData[4];
//	}

	pState->vel_accel[0] = fusionEKF.kf.x.pData[3];
	pState->vel_accel[1] = fusionEKF.kf.x.pData[4];

	pState->pos[0] = fusionEKF.kf.x.pData[0]; // position X
	pState->pos[1] = fusionEKF.kf.x.pData[1]; // position Y
	pState->pos[2] = fusionEKF.kf.x.pData[2]; // angular position

	pState->vel[0] = fusionEKF.kf.x.pData[3]; // linear velocity X
	pState->vel[1] = fusionEKF.kf.x.pData[4]; // linear velocity Y
	pState->vel[2] = stateNow.accGyr[2]; // angular velocity

//	pState->global_accel[0] = accelGlobal[0];
//	pState->global_accel[1] = accelGlobal[1];
//	pState->global_vel[0] = velGlobal[0];
//	pState->global_vel[1] = velGlobal[1];
}

void Vector2fTurnLocal2Global(float globalOrientation, float localX, float localY, float* pGlobalX, float* pGlobalY)
{
	float localToGlobalAngle = -PI / 2.0f + globalOrientation;
	float sinA = arm_sin_f32(localToGlobalAngle);
	float cosA = arm_cos_f32(localToGlobalAngle);

	// turn to global system
	*pGlobalX = localX * cosA - localY * sinA;
	*pGlobalY = localY * cosA + localX * sinA;
}

static uint8_t isVisionSampleValid(const float* pVisPos, const float* pLastPos)
{
	// validate sample, outlier rejection
//	uint32_t measTime = HAL_GetTick()-tDelayUs;
	uint32_t new_sample_time = HAL_GetTick();
	float visionDt = (new_sample_time-fusionEKF.vision.timeLastValidSample)*1e-3f;

//	float searchRadiusXY = 100.0f*fusionEKF.pConfig->visNoiseXY + fusionEKF.pConfig->outlierMaxVelXY*visionDt;
//	float searchRadiusW = 2.0f*fusionEKF.pConfig->visNoiseW + fusionEKF.pConfig->outlierMaxVelW*visionDt;

	float searchRadiusXY = fusionEKF.pConfig->outlierMaxVelXY*visionDt;
	float searchRadiusW = fusionEKF.pConfig->outlierMaxVelW*visionDt;

	float diffX = (pVisPos[0] - pLastPos[0])*1e-3;
	float diffY = (pVisPos[1] - pLastPos[1])*1e-3;
	float diffXY = sqrtf(diffX*diffX+diffY*diffY);
//	float diffW = AngleNormalize(pVisPos[2] - AngleNormalize(pLastPos[2]));

	if(diffXY > searchRadiusXY)
	{
		// this is an invalid sample, vel is impossible
		return 0;
	}

	memcpy(pLastPos, pVisPos, sizeof(float)*3);

	fusionEKF.vision.timeLastValidSample = new_sample_time;

	return 1;
}

void FusionEKFInit(FusionEKFConfig* pConfigKF)
{
	fusionEKF.pConfig = pConfigKF;
	initEKF();
}

static void loadNoiseCovariancesFromConfig()
{
	if(fusionEKF.kf.Ex.pData) // pData is null when fusionEKF has not been initialized yet
	{
		MAT_ELEMENT(fusionEKF.kf.Ex, 0, 0) = fusionEKF.pConfig->posNoiseXY*fusionEKF.pConfig->posNoiseXY;
		MAT_ELEMENT(fusionEKF.kf.Ex, 1, 1) = fusionEKF.pConfig->posNoiseXY*fusionEKF.pConfig->posNoiseXY;
		MAT_ELEMENT(fusionEKF.kf.Ex, 2, 2) = fusionEKF.pConfig->posNoiseW *fusionEKF.pConfig->posNoiseW;
		MAT_ELEMENT(fusionEKF.kf.Ex, 3, 3) = fusionEKF.pConfig->velNoiseXY*fusionEKF.pConfig->velNoiseXY;
		MAT_ELEMENT(fusionEKF.kf.Ex, 4, 4) = fusionEKF.pConfig->velNoiseXY*fusionEKF.pConfig->velNoiseXY;

		MAT_ELEMENT(fusionEKF.kf.Ez, 0, 0) = fusionEKF.pConfig->visNoiseXY *fusionEKF.pConfig->visNoiseXY;
		MAT_ELEMENT(fusionEKF.kf.Ez, 1, 1) = fusionEKF.pConfig->visNoiseXY *fusionEKF.pConfig->visNoiseXY;
		MAT_ELEMENT(fusionEKF.kf.Ez, 2, 2) = fusionEKF.pConfig->visNoiseW  *fusionEKF.pConfig->visNoiseW;
		MAT_ELEMENT(fusionEKF.kf.Ez, 3, 3) = fusionEKF.pConfig->visNoiseVel*fusionEKF.pConfig->visNoiseVel;
		MAT_ELEMENT(fusionEKF.kf.Ez, 4, 4) = fusionEKF.pConfig->visNoiseVel*fusionEKF.pConfig->visNoiseVel;
	}
}

static void initEKF()
{
//	KFInit(&fusionEKF.kf, 5, 3, 3, fusionEKF.ekfData);
	KFInit(&fusionEKF.kf, 5, 3, 5, fusionEKF.ekfData);

	arm_mat_scale_f32(&fusionEKF.kf.Sigma, 0.001f, &fusionEKF.kf.Sigma);

	fusionEKF.kf.pState = &ekfStateFunc;
//	fusionEKF.kf.pState = &ekfStateFunc_encoder;
//	fusionEKF.kf.pState = &ekfStateFunc_model;

	fusionEKF.kf.pStateJacobian = &ekfStateJacobianFunc;
//	fusionEKF.kf.pStateJacobian = &ekfStateJacobianFunc_encoder;
//	fusionEKF.kf.pStateJacobian = &ekfStateJacobianFunc_model;

//	fusionEKF.kf.pMeas = &ekfMeasFuncPoseState;
	fusionEKF.kf.pMeas = &ekfMeasFuncFullState;

//	fusionEKF.kf.pMeasJacobian = &ekfMeasJacobianFuncPoseState;
	fusionEKF.kf.pMeasJacobian = &ekfMeasJacobianFuncFullState;

	fusionEKF.first_vision_meas = true;

	fusionEKF.predict_now = true;

	fusionEKF.update_counter = 0;
	fusionEKF.predict_counter = 0;

	arm_mat_identity_f32(&fusionEKF.kf.Ex);
	arm_mat_identity_f32(&fusionEKF.kf.Ez);

	loadNoiseCovariancesFromConfig();
}

void FusionEKFSetState(const float* pPos)
{
	for (uint32_t i = 0; i < 3; i++)
	{
		memcpy(fusionEKF.kf.x.pData+i, pPos+i, sizeof(float));
	}

	memcpy(fusionEKF.kf.x.pData+3, 0, sizeof(float));
	memcpy(fusionEKF.kf.x.pData+4, 0, sizeof(float));
//	memcpy(fusionEKF.encGyrPos, pPos, sizeof(float)*3);

//	for(uint32_t i = 0; i < FUSION_EKF_MAX_DELAY; i++)
//	{
//		memcpy(fusionEKF.timeSlots[i].insState, pPos, sizeof(float)*3);
//		memcpy(fusionEKF.timeSlots[i].insState+3, pVel, sizeof(float)*2);
//		memcpy(fusionEKF.timeSlots[i].meas.pos, pPos, sizeof(float)*3);
//	}
}

static float visionMultiTurnCorrection(float visionOrientation)
{
	if(visionOrientation < -2.0f && fusionEKF.vision.lastOrient > 2.0f)
		++fusionEKF.vision.turns;

	if(visionOrientation > 2.0f && fusionEKF.vision.lastOrient < -2.0f)
		--fusionEKF.vision.turns;

	fusionEKF.vision.lastOrient = visionOrientation;

	return visionOrientation + fusionEKF.vision.turns*2.0f*M_PI;
}

void ekfStateFunc_model(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, float dt)
{
	float vel_x = MAT_ELEMENT(*pU, 0, 0);
	float vel_y = MAT_ELEMENT(*pU, 1, 0);
	float vel_theta = MAT_ELEMENT(*pU, 2, 0);

	float p_x = MAT_ELEMENT(*pX, 0, 0);
	float p_y = MAT_ELEMENT(*pX, 1, 0);
	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

	float a = -M_PI_2 + p_w;

	float px1 = p_x + (arm_cos_f32(a)*v_x-arm_sin_f32(a)*v_y)*dt;
	float py1 = p_y + (arm_sin_f32(a)*v_x+arm_cos_f32(a)*v_y)*dt;
	float vx1 = vel_x;
	float vy1 = vel_y;
	float pw1 = p_w + vel_theta*dt;

	MAT_ELEMENT(*pX, 0, 0) = px1;
	MAT_ELEMENT(*pX, 1, 0) = py1;
	MAT_ELEMENT(*pX, 2, 0) = pw1;
	MAT_ELEMENT(*pX, 3, 0) = vx1;
	MAT_ELEMENT(*pX, 4, 0) = vy1;
}

void ekfStateFunc_encoder(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, float dt)
{
	float vel_x = MAT_ELEMENT(*pU, 0, 0);
	float vel_y = MAT_ELEMENT(*pU, 1, 0);
	float vel_theta = MAT_ELEMENT(*pU, 2, 0);

	float p_x = MAT_ELEMENT(*pX, 0, 0);
	float p_y = MAT_ELEMENT(*pX, 1, 0);
	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

	float a = -M_PI_2 + p_w;

	float px1 = p_x + (arm_cos_f32(a)*v_x-arm_sin_f32(a)*v_y)*dt;
	float py1 = p_y + (arm_sin_f32(a)*v_x+arm_cos_f32(a)*v_y)*dt;
	float vx1 = vel_x;
	float vy1 = vel_y;
	float pw1 = p_w + vel_theta*dt;

	MAT_ELEMENT(*pX, 0, 0) = px1;
	MAT_ELEMENT(*pX, 1, 0) = py1;
	MAT_ELEMENT(*pX, 2, 0) = pw1;
	MAT_ELEMENT(*pX, 3, 0) = vx1;
	MAT_ELEMENT(*pX, 4, 0) = vy1;
}

void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, float dt)
{

	float acc_x = MAT_ELEMENT(*pU, 0, 0);
	float acc_y = MAT_ELEMENT(*pU, 1, 0);
	float gyr_w = MAT_ELEMENT(*pU, 2, 0);

	float p_x = MAT_ELEMENT(*pX, 0, 0);
	float p_y = MAT_ELEMENT(*pX, 1, 0);
	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

	float v_w = gyr_w;
	float a = -M_PI_2 + p_w;
	float a_x = acc_x;
	float a_y = acc_y;

	float px1 = p_x + (arm_cos_f32(a)*v_x-arm_sin_f32(a)*v_y)*dt + (arm_cos_f32(a)*a_x-arm_sin_f32(a)*a_y)*0.5f*dt*dt;
	float py1 = p_y + (arm_sin_f32(a)*v_x+arm_cos_f32(a)*v_y)*dt + (arm_sin_f32(a)*a_x+arm_cos_f32(a)*a_y)*0.5f*dt*dt;
	float vx1 = v_x + a_x*dt;
	float vy1 = v_y + a_y*dt;
	float pw1 = p_w + v_w*dt;

	MAT_ELEMENT(*pX, 0, 0) = px1;
	MAT_ELEMENT(*pX, 1, 0) = py1;
	MAT_ELEMENT(*pX, 2, 0) = pw1;
	MAT_ELEMENT(*pX, 3, 0) = vx1;
	MAT_ELEMENT(*pX, 4, 0) = vy1;
}

static void ekfStateJacobianFunc_model(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF, float dt)
{
	float vel_x = MAT_ELEMENT(*pU, 0, 0);
	float vel_y = MAT_ELEMENT(*pU, 1, 0);
	float vel_theta = MAT_ELEMENT(*pU, 2, 0);

	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

	arm_mat_identity_f32(pF);

	MAT_ELEMENT(*pF, 3, 3) = 0.0;
	MAT_ELEMENT(*pF, 4, 4) = 0.0;

	MAT_ELEMENT(*pF, 2, 2) = 1.0;

	MAT_ELEMENT(*pF, 0, 2) = arm_cos_f32(p_w) * v_x * dt - arm_sin_f32(p_w) * v_y * dt;
	MAT_ELEMENT(*pF, 0, 3) = arm_sin_f32(p_w) * dt;
	MAT_ELEMENT(*pF, 0, 4) = arm_cos_f32(p_w) * dt;

	MAT_ELEMENT(*pF, 1, 2) = arm_sin_f32(p_w) * v_x * dt + arm_cos_f32(p_w) * v_y * dt;
	MAT_ELEMENT(*pF, 1, 3) = -arm_cos_f32(p_w) * dt;
	MAT_ELEMENT(*pF, 1, 4) = arm_sin_f32(p_w) * dt;
}

static void ekfStateJacobianFunc_encoder(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF, float dt)
{
	float vel_x = MAT_ELEMENT(*pU, 0, 0);
	float vel_y = MAT_ELEMENT(*pU, 1, 0);
	float vel_theta = MAT_ELEMENT(*pU, 2, 0);

	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

	arm_mat_identity_f32(pF);

	MAT_ELEMENT(*pF, 3, 3) = 0.0;
	MAT_ELEMENT(*pF, 4, 4) = 0.0;

	MAT_ELEMENT(*pF, 2, 2) = 1.0;

	MAT_ELEMENT(*pF, 0, 2) = arm_cos_f32(p_w) * v_x * dt - arm_sin_f32(p_w) * v_y * dt;
	MAT_ELEMENT(*pF, 0, 3) = arm_sin_f32(p_w) * dt;
	MAT_ELEMENT(*pF, 0, 4) = arm_cos_f32(p_w) * dt;

	MAT_ELEMENT(*pF, 1, 2) = arm_sin_f32(p_w) * v_x * dt + arm_cos_f32(p_w) * v_y * dt;
	MAT_ELEMENT(*pF, 1, 3) = -arm_cos_f32(p_w) * dt;
	MAT_ELEMENT(*pF, 1, 4) = arm_sin_f32(p_w) * dt;
}

static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF, float dt)
{
	float acc_x = MAT_ELEMENT(*pU, 0, 0);
	float acc_y = MAT_ELEMENT(*pU, 1, 0);
	float gyr_w = MAT_ELEMENT(*pU, 2, 0);

	float p_w = MAT_ELEMENT(*pX, 2, 0);
	float v_x = MAT_ELEMENT(*pX, 3, 0);
	float v_y = MAT_ELEMENT(*pX, 4, 0);

	arm_mat_identity_f32(pF);

	MAT_ELEMENT(*pF, 0, 2) = arm_cos_f32(p_w) * v_x * dt - arm_sin_f32(p_w) * v_y * dt + arm_cos_f32(p_w) * acc_x * dt * dt * 0.5f - arm_sin_f32(p_w) * acc_y * dt * dt * 0.5f;
	MAT_ELEMENT(*pF, 0, 3) = arm_sin_f32(p_w) * dt;
	MAT_ELEMENT(*pF, 0, 4) = arm_cos_f32(p_w) * dt;

	MAT_ELEMENT(*pF, 1, 2) = arm_sin_f32(p_w) * v_x * dt + arm_cos_f32(p_w) * v_y * dt + arm_sin_f32(p_w) * acc_x * dt * dt * 0.5f + arm_cos_f32(p_w) * acc_y * dt * dt * 0.5f;
	MAT_ELEMENT(*pF, 1, 3) = -arm_cos_f32(p_w) * dt;
	MAT_ELEMENT(*pF, 1, 4) = arm_sin_f32(p_w) * dt;
}

static void ekfMeasFuncPoseState(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY)
{
	memcpy(pY->pData, pX->pData, sizeof(float)*3);
}

static void ekfMeasFuncFullState(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY)
{
	memcpy(pY->pData, pX->pData, sizeof(float)*5);
}

static void ekfMeasJacobianFuncFullState(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH)
{
	(void)pX;

	arm_mat_zero_f32(pH);
	MAT_ELEMENT(*pH, 0, 0) = 1.0f;
	MAT_ELEMENT(*pH, 1, 1) = 1.0f;
	MAT_ELEMENT(*pH, 2, 2) = 1.0f;
	MAT_ELEMENT(*pH, 3, 3) = 1.0f;
	MAT_ELEMENT(*pH, 4, 4) = 1.0f;
}

static void ekfMeasJacobianFuncPoseState(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH)
{
	(void)pX;

	arm_mat_zero_f32(pH);
	MAT_ELEMENT(*pH, 0, 0) = 1.0f;
	MAT_ELEMENT(*pH, 1, 1) = 1.0f;
	MAT_ELEMENT(*pH, 2, 2) = 1.0f;
}

float AngleNormalize(float a)
{
	return mod(a + M_PI, M_TWOPI) - M_PI;
}

float mod(float x, float y)
{
	if(y == 0.0f)
		return x;

	float m = x - y * floorf(x / y);

	// handle boundary cases resulted from floating-point cut off:
	if(y > 0) // modulo range: [0..y)
	{
		if(m >= y)
			return 0;

		if(m < 0)
		{
			if(y + m == y)
				return 0;
			else
				return y + m;
		}
	}
	else // modulo range: (y..0]
	{
		if(m <= y)
			return 0;

		if(m > 0)
		{
			if(y + m == y)
				return 0;
			else
				return y + m;
		}
	}

	return m;
}
