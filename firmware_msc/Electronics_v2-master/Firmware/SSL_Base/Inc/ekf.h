/*
 * ekf.h
 *
 *  Created on: May 31, 2024
 *      Author: joaov
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#define ARM_MATH_CM4
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define KF_SIZE_A(f)		(f*f)
#define KF_SIZE_B(f,g)		(f*g)
#define KF_SIZE_C(h,f)		(h*f)
#define KF_SIZE_EX(f)		(f*f)
#define KF_SIZE_EZ(h)		(h*h)
#define KF_SIZE_X(f)		(f)
#define KF_SIZE_SIGMA(f)	(f*f)
#define KF_SIZE_K(f,h)		(f*h)
#define KF_SIZE_U(g)		(g)
#define KF_SIZE_Z(h)		(h)
#define KF_SIZE_MAX(max)	(max*max)

// state vector (x) rows: f
// control vector (u) rows: g
// sensor vector (z) rows: h
#define KF_DATA_SIZE(f,g,h,max) \
	(KF_SIZE_A(f)+KF_SIZE_B(f,g)+KF_SIZE_C(h,f)+KF_SIZE_EX(f) \
	+KF_SIZE_EZ(h)+KF_SIZE_X(f)+KF_SIZE_SIGMA(f)+KF_SIZE_K(f,h)+KF_SIZE_U(g) \
	+KF_SIZE_Z(h)+KF_SIZE_MAX(max)*3+KF_SIZE_X(f))

typedef void(*KFStateFunc)(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
//typedef void(*EKFStateJacobianFunc)(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF);
//typedef void(*EKFMeasFunc)(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
//typedef void(*EKFMeasJacobianFunc)(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);

typedef struct _KF
{
	uint16_t f;
	uint16_t g;
	uint16_t h;

	KFStateFunc pState;

	// user matrices
	arm_matrix_instance_f32 A;		// (f x f)
	arm_matrix_instance_f32 B;		// (f x g)
	arm_matrix_instance_f32 C;		// (h x f)
	arm_matrix_instance_f32 Ex;		// (f x f)
	arm_matrix_instance_f32 Ez;		// (h x h)

	// internal matrices
	arm_matrix_instance_f32 x;		// state (f x 1)
	arm_matrix_instance_f32 Sigma;	// uncertainty (f x f)
	arm_matrix_instance_f32 K;		// Kalman gain (f x h)

	// command input
	arm_matrix_instance_f32 u;		// (g x 1)

	// sensor input
	arm_matrix_instance_f32 z;		// (h x 1)

	// temporary calculation matrices
	arm_matrix_instance_f32 tmp1;
	arm_matrix_instance_f32 tmp2;
	arm_matrix_instance_f32 tmp3;

	// angle selector
	uint32_t* pNormalizeAngle;
} KF;

void KFInit(KF* pKF, uint16_t f, uint16_t g, uint16_t h, float* pData);
void KFSetOrientationComponent(KF* pKF, uint32_t stateIndex);
void KFPredict(KF* pKF);
void KFUpdate(KF* pKF);
arm_status arm_mat_identity_f32(arm_matrix_instance_f32* pMat);
void arm_mat_zero_f32(arm_matrix_instance_f32* pMat);
arm_status arm_mat_inv_2x2_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);
arm_status arm_mat_inv_3x3_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);

#endif /* INC_EKF_H_ */
