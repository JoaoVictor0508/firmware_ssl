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
#include "main.h"

void arm_mat_copy_f32(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst)
{
	pDst->numCols = pSrc->numCols;
	pDst->numRows = pSrc->numRows;

	memcpy(pDst->pData, pSrc->pData, sizeof(float) * pSrc->numCols * pSrc->numRows);
}

arm_status arm_mat_identity_f32(arm_matrix_instance_f32* pMat)
{
#ifdef ARM_MATH_MATRIX_CHECK
	if(pMat->numCols != pMat->numRows)
	return ARM_MAT_SIZE_MISMATCH;
#endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */

	arm_mat_zero_f32(pMat);

	for(uint16_t i = 0; i < pMat->numCols; i++)
		pMat->pData[i * pMat->numCols + i] = 1.0f;

	return ARM_MATH_SUCCESS;
}

void arm_mat_zero_f32(arm_matrix_instance_f32* pMat)
{
	memset(pMat->pData, 0, sizeof(float) * pMat->numCols * pMat->numRows);
}

#define DET2(a, b, c, d) (a*d-b*c)

arm_status arm_mat_inv_2x2_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
#ifdef ARM_MATH_MATRIX_CHECK
	if(A->numCols != 2 || A->numRows != 2)
		return ARM_MAT_SIZE_MISMATCH;
#endif

	float det = DET2(A->pData[0], A->pData[1], A->pData[2], A->pData[3]);
	if(det == 0.0f)
		return ARM_MATH_SINGULAR;

	float f = 1.0f/det;

	B->pData[0] =  f*A->pData[3];
	B->pData[1] = -f*A->pData[1];
	B->pData[2] = -f*A->pData[2];
	B->pData[3] =  f*A->pData[0];

	return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inv_3x3_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
#ifdef ARM_MATH_MATRIX_CHECK
	if(A->numCols != 3 || A->numRows != 3)
		return ARM_MAT_SIZE_MISMATCH;
#endif

	const float* a = A->pData;
	const float* b = A->pData+3;
	const float* c = A->pData+6;

	float det = a[0]*b[1]*c[2] - a[0]*b[2]*c[1] - a[1]*b[0]*c[2] + a[1]*b[2]*c[0] + a[2]*b[0]*c[1] - a[2]*b[1]*c[0];
	if(det == 0.0f)
		return ARM_MATH_SINGULAR;

	float f = 1.0f/det;

	a = A->pData;

	B->pData[0] = f*DET2(a[4], a[5], a[7], a[8]);
	B->pData[1] = f*DET2(a[2], a[1], a[8], a[7]);
	B->pData[2] = f*DET2(a[1], a[2], a[4], a[5]);
	B->pData[3] = f*DET2(a[5], a[3], a[8], a[6]);
	B->pData[4] = f*DET2(a[0], a[2], a[6], a[8]);
	B->pData[5] = f*DET2(a[2], a[1], a[5], a[3]);
	B->pData[6] = f*DET2(a[3], a[4], a[6], a[7]);
	B->pData[7] = f*DET2(a[1], a[0], a[7], a[6]);
	B->pData[8] = f*DET2(a[0], a[1], a[3], a[4]);

	return ARM_MATH_SUCCESS;
}

void KFInit(KF* pKF, uint16_t f, uint16_t g, uint16_t h, float* pData)
{
	uint16_t max = f;
	if(g > max)
		max = g;
	if(h > max)
		max = h;

	memset(pData, 0, KF_DATA_SIZE(f, g, h, max)*sizeof(float));

	pKF->f = f;
	pKF->g = g;
	pKF->h = h;

	pKF->A.numRows = f;
	pKF->A.numCols = f;
	pKF->A.pData = pData;
	pData += KF_SIZE_A(f);

	pKF->B.numRows = f;
	pKF->B.numCols = g;
	pKF->B.pData = pData;
	pData += KF_SIZE_B(f, g);

	pKF->C.numRows = h;
	pKF->C.numCols = f;
	pKF->C.pData = pData;
	pData += KF_SIZE_C(h, f);

	pKF->Ex.numRows = f;
	pKF->Ex.numCols = f;
	pKF->Ex.pData = pData;
	pData += KF_SIZE_EX(f);

	pKF->Ez.numRows = h;
	pKF->Ez.numCols = h;
	pKF->Ez.pData = pData;
	pData += KF_SIZE_EZ(h);

	pKF->x.numRows = f;
	pKF->x.numCols = 1;
	pKF->x.pData = pData;
	pData += KF_SIZE_X(f);

	pKF->Sigma.numRows = f;
	pKF->Sigma.numCols = f;
	pKF->Sigma.pData = pData;
	pData += KF_SIZE_SIGMA(f);

	pKF->K.numRows = f;
	pKF->K.numCols = h;
	pKF->K.pData = pData;
	pData += KF_SIZE_K(f,h);

	pKF->u.numRows = g;
	pKF->u.numCols = 1;
	pKF->u.pData = pData;
	pData += KF_SIZE_U(g);

	pKF->z.numRows = h;
	pKF->z.numCols = 1;
	pKF->z.pData = pData;
	pData += KF_SIZE_Z(h);

	pKF->tmp1.numRows = max;
	pKF->tmp1.numCols = max;
	pKF->tmp1.pData = pData;
	pData += KF_SIZE_MAX(max);

	pKF->tmp2.numRows = max;
	pKF->tmp2.numCols = max;
	pKF->tmp2.pData = pData;
	pData += KF_SIZE_MAX(max);

	pKF->tmp3.numRows = max;
	pKF->tmp3.numCols = max;
	pKF->tmp3.pData = pData;
	pData += KF_SIZE_MAX(max);

	pKF->pNormalizeAngle = (uint32_t*)pData;
	pData += KF_SIZE_X(f);

	arm_mat_identity_f32(&pKF->Sigma);
}

void KFPredict(KF* pKF, float dt)
{
	// >>> State prediction
	// A = jacobian(x+1,x)
	(*pKF->pStateJacobian)(&pKF->x, &pKF->u, &pKF->A, dt);
	(*pKF->pState)(&pKF->x, &pKF->u, dt);

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

void KFUpdate(KF* pKF)
{
//	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

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
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

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

	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
}




