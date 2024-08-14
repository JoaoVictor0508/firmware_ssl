/*
 * ekf.c
 *
 *  Created on: May 31, 2024
 *      Author: joaov
 */

#define ARM_MATH_CM4
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "ekf.h"
#include "calibrate_imu.h"


//void EKFStateFunc(arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pU)
//{
//	const float dt = 0.001f;
//
//	float gyr_w = MAT_ELEMENT(*pU, 0, 0);
//	float acc_x = MAT_ELEMENT(*pU, 1, 0);
//	float acc_y = MAT_ELEMENT(*pU, 2, 0);
//
//	float p_x = MAT_ELEMENT(*pX, 0, 0);
//	float p_y = MAT_ELEMENT(*pX, 1, 0);
//	float p_w = MAT_ELEMENT(*pX, 2, 0);
//	float v_x = MAT_ELEMENT(*pX, 3, 0);
//	float v_y = MAT_ELEMENT(*pX, 4, 0);
//
//	float v_w = gyr_w;
//	float a = -M_PI_2 + p_w + dt*v_w*0.5f;
//	float a_x = acc_x;
//	float a_y = acc_y;
////	float a_x = acc_x + v_y*v_w; // Tigers method with centrifugal force
////	float a_y = acc_y - v_x*v_w; // Tigers method with centrifugal force
//
//	float px1 = p_x + (arm_cos_f32(a)*v_x-arm_sin_f32(a)*v_y)*dt + (arm_cos_f32(a)*a_x-arm_sin_f32(a)*a_y)*0.5f*dt*dt;
//	float py1 = p_y + (arm_sin_f32(a)*v_x+arm_cos_f32(a)*v_y)*dt + (arm_sin_f32(a)*a_x+arm_cos_f32(a)*a_y)*0.5f*dt*dt;
//	float vx1 = v_x + a_x*dt;
//	float vy1 = v_y + a_y*dt;
//	float pw1 = p_w + v_w*dt;
//
////	pX->pData[0][0] = px1;
////	*pX->pData[1][0] = py1;
////	*pX->pData[2][0] = pw1;
////	*pX->pData[3][0] = vx1;
////	*pX->pData[4][0] = vy1;
//	MAT_ELEMENT(*pX, 0, 0) = px1; //MAT_ELEMENT ONLY WORKING BECAUSE OF CALIBRATE_IMU INCLUDE (?)
//	MAT_ELEMENT(*pX, 1, 0) = py1;
//	MAT_ELEMENT(*pX, 2, 0) = pw1;
//	MAT_ELEMENT(*pX, 3, 0) = vx1;
//	MAT_ELEMENT(*pX, 4, 0) = vy1;
//}

void EKFPredict(EKF* pKF)
{
	// >>> State prediction
	// A = jacobian(x+1,x)
	(*pKF->pStateJacobian)(&pKF->x, &pKF->u, &pKF->A);
	(*pKF->pState)(&pKF->x, &pKF->u);

	// >>> Sigma = A*Sigma*A^T + Ex
	// tmp1 = A*Sigma (f x f)
	pKF->tmp1.numRows = pKF->f;
	pKF->tmp1.numCols = pKF->f;
	arm_mat_mult_f32(&pKF->A, &pKF->Sigma, &pKF->tmp1);
	// tmp2 = A^T (f x f)
	pKF->tmp2.numRows = pKF->f;
	pKF->tmp2.numCols = pKF->f;
	arm_mat_trans_f32(&pKF->A, &pKF->tmp2);
	// Sigma = tmp1*tmp2 (f x f)
	arm_mat_mult_f32(&pKF->tmp1, &pKF->tmp2, &pKF->Sigma);
	// Sigma = Sigma + Ex (f x f)
	arm_mat_add_f32(&pKF->Sigma, &pKF->Ex, &pKF->Sigma);
}

void EKFUpdate(EKF* pKF)
{
	// >>> H = jacobian(z,x)
	(*pKF->pMeasJacobian)(&pKF->x, &pKF->C);

	// >>> K = Sigma*C^T*(C*Sigma*C^T+Ez)^-1
	// tmp1 = C^T (f x h)
	pKF->tmp1.numRows = pKF->f;
	pKF->tmp1.numCols = pKF->h;
	arm_mat_trans_f32(&pKF->C, &pKF->tmp1);
	// tmp2 = Sigma*tmp1 (f x h)
	pKF->tmp2.numRows = pKF->f;
	pKF->tmp2.numCols = pKF->h;
	arm_mat_mult_f32(&pKF->Sigma, &pKF->tmp1, &pKF->tmp2);
	// tmp3 = C*tmp2 (h x h)
	pKF->tmp3.numRows = pKF->h;
	pKF->tmp3.numCols = pKF->h;
	arm_mat_mult_f32(&pKF->C, &pKF->tmp2, &pKF->tmp3);
	// tmp3 = tmp3+Ez (h x h)
	arm_mat_add_f32(&pKF->tmp3, &pKF->Ez, &pKF->tmp3);
	// tmp1 = tmp3^-1 (h x h)
	pKF->tmp1.numRows = pKF->h;
	pKF->tmp1.numCols = pKF->h;
	arm_status invStat;
	switch(pKF->h)
	{
		case 1:
			if(pKF->tmp3.pData[0] == 0.0f)
			{
				invStat = ARM_MATH_SINGULAR;
			}
			else
			{
				pKF->tmp1.pData[0] = 1.0f/pKF->tmp3.pData[0];
				invStat = ARM_MATH_SUCCESS;
			}
			break;
		case 2:
			invStat = arm_mat_inv_2x2_f32(&pKF->tmp3, &pKF->tmp1);
			break;
		case 3:
			invStat = arm_mat_inv_3x3_f32(&pKF->tmp3, &pKF->tmp1);
			break;
		default:
			invStat = arm_mat_inverse_f32(&pKF->tmp3, &pKF->tmp1);
			break;
	}
	if(invStat != ARM_MATH_SUCCESS)
//		LogErrorC("Matrix inverse error", invStat);
	// K = tmp2*tmp1 (f x h)
	arm_mat_mult_f32(&pKF->tmp2, &pKF->tmp1, &pKF->K);

	// >>> x = x + K*(z - pMeas(x))
	// tmp1 = pMeas(x) (h x 1)
	pKF->tmp1.numRows = pKF->h;
	pKF->tmp1.numCols = 1;
	(*pKF->pMeas)(&pKF->x, &pKF->tmp1);
	// tmp2 = z - tmp1 (h x 1)
	pKF->tmp2.numRows = pKF->h;
	pKF->tmp2.numCols = 1;
	arm_mat_sub_f32(&pKF->z, &pKF->tmp1, &pKF->tmp2);
	// tmp3 = K*tmp2 (f x 1)
	pKF->tmp3.numRows = pKF->f;
	pKF->tmp3.numCols = 1;
	arm_mat_mult_f32(&pKF->K, &pKF->tmp2, &pKF->tmp3);
	// x = x + tmp3 (f x 1)
	arm_mat_add_f32(&pKF->x, &pKF->tmp3, &pKF->x);

	// >>> Sigma = (I - K*C)*Sigma
	// tmp1 = K*C (f x f)
	pKF->tmp1.numRows = pKF->f;
	pKF->tmp1.numCols = pKF->f;
	arm_mat_mult_f32(&pKF->K, &pKF->C, &pKF->tmp1);
	// tmp2 = I (f x f)
	pKF->tmp2.numRows = pKF->f;
	pKF->tmp2.numCols = pKF->f;
	arm_mat_identity_f32(&pKF->tmp2);
	// tmp2 = tmp2 - tmp1 (f x f)
	arm_mat_sub_f32(&pKF->tmp2, &pKF->tmp1, &pKF->tmp2);
	// tmp3 = Sigma (f x f)
	arm_mat_copy_f32(&pKF->Sigma, &pKF->tmp3);
	// Sigma = tmp2*tmp3 (f x f)
	arm_mat_mult_f32(&pKF->tmp2, &pKF->tmp3, &pKF->Sigma);
}




