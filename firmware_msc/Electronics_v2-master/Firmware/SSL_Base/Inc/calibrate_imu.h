/*
 * calibrate_imu.h
 *
 *  Created on: May 25, 2024
 *      Author: joaov
 */

#ifndef INC_CALIBRATE_IMU_H_
#define INC_CALIBRATE_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_MATH_CM4
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "lag_element.h"

#define MAT_ELEMENT(mat,r,c) ((mat).pData[(mat).numCols*(r)+(c)])

void CalibIMU();
void CalibIMU_Init();

arm_status arm_mat_pinv(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);

#ifdef __cplusplus
}
#endif

#endif /* INC_CALIBRATE_IMU_H_ */
