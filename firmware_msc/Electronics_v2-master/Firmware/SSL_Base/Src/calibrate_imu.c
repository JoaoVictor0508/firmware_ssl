/*
 * calibrate_imu.c
 *
 *  Created on: May 24, 2024
 *      Author: joaov
 */
//#define ARM_MATH_CM4
//#include "stm32f4xx_hal.h"
//#include "arm_math.h"
#include "calibrate_imu.h"
#include "signal_statistics.h"
#include "l3gd20.h"
#include "lsm303dlhc.h"
#include <stdio.h>
//extern void initialise_monitor_handles(void);

#define SVD_MAX_SIZE 10

#define SIGNF(a, b) ((b) >= 0.0 ? fabsf(a) : -fabsf(a))
#define MAX(x,y) ((x)>(y)?(x):(y))
static float PYTHAGF(float a, float b);

#define IMU_CALIB_NUM_SAMPLES 1
#define Accel_Get_Data_Wrong 3000

#define CTRL_DELTA_T_IN_US 1000
#define CTRL_DELTA_T (CTRL_DELTA_T_IN_US*1e-6f)

uint16_t samples_view = 0;
uint16_t repeat_view = 0;

LagElementPT1 lagAccel[2];

FILE *fp, *file_1;
char p[100], p_1[100];
char *c, *c_1;


arm_status arm_mat_svd(arm_matrix_instance_f32* U, arm_matrix_instance_f32* S, arm_matrix_instance_f32* V)
{
    int16_t flag, i, its, j, jj, k;
    float c, f, h, s, x, y, z;
    float anorm = 0.0, g = 0.0, scale = 0.0;
    float rv1[SVD_MAX_SIZE];
    int16_t l = 0;
    int16_t nm = 0;
    uint16_t m = U->numRows;
    uint16_t n = U->numCols;

    if (m < n || n > SVD_MAX_SIZE)
        return ARM_MATH_SIZE_MISMATCH;

/* Householder reduction to bidiagonal form */
    for (i = 0; i < n; i++)
    {
        /* left-hand reduction */
        l = i + 1;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m)
        {
            for (k = i; k < m; k++)
                scale += fabsf(MAT_ELEMENT(*U,k,i));
            if (scale)
            {
                for (k = i; k < m; k++)
                {
                    MAT_ELEMENT(*U,k,i) = (MAT_ELEMENT(*U,k,i)/scale);
                    s += (MAT_ELEMENT(*U,k,i) * MAT_ELEMENT(*U,k,i));
                }
                f = MAT_ELEMENT(*U,i,i);
                g = -SIGNF(sqrtf(s), f);
                h = f * g - s;
                MAT_ELEMENT(*U,i,i) = (f - g);
                if (i != n - 1)
                {
                    for (j = l; j < n; j++)
                    {
                        for (s = 0.0, k = i; k < m; k++)
                            s += (MAT_ELEMENT(*U,k,i) * MAT_ELEMENT(*U,k,j));
                        f = s / h;
                        for (k = i; k < m; k++)
                            MAT_ELEMENT(*U,k,j) += (f * MAT_ELEMENT(*U,k,i));
                    }
                }
                for (k = i; k < m; k++)
                    MAT_ELEMENT(*U,k,i) = (MAT_ELEMENT(*U,k,i)*scale);
            }
        }
        MAT_ELEMENT(*S,i,i) = (scale * g);

        /* right-hand reduction */
        g = s = scale = 0.0;
        if (i < m && i != n - 1)
        {
            for (k = l; k < n; k++)
                scale += fabsf(MAT_ELEMENT(*U,i,k));
            if (scale)
            {
                for (k = l; k < n; k++)
                {
                    MAT_ELEMENT(*U,i,k) = (MAT_ELEMENT(*U,i,k)/scale);
                    s += (MAT_ELEMENT(*U,i,k) * MAT_ELEMENT(*U,i,k));
                }
                f = MAT_ELEMENT(*U,i,l);
                g = -SIGNF(sqrtf(s), f);
                h = f * g - s;
                MAT_ELEMENT(*U,i,l) = (f - g);
                for (k = l; k < n; k++)
                    rv1[k] = MAT_ELEMENT(*U,i,k) / h;
                if (i != m - 1)
                {
                    for (j = l; j < m; j++)
                    {
                        for (s = 0.0, k = l; k < n; k++)
                            s += (MAT_ELEMENT(*U,j,k) * MAT_ELEMENT(*U,i,k));
                        for (k = l; k < n; k++)
                            MAT_ELEMENT(*U,j,k) += (s * rv1[k]);
                    }
                }
                for (k = l; k < n; k++)
                    MAT_ELEMENT(*U,i,k) = (MAT_ELEMENT(*U,i,k)*scale);
            }
        }
        anorm = MAX(anorm, (fabsf(MAT_ELEMENT(*S,i,i)) + fabsf(rv1[i])));
    }

    /* accumulate the right-hand transformation */
    for (i = n - 1; i >= 0; i--)
    {
        if (i < n - 1)
        {
            if (g)
            {
                for (j = l; j < n; j++)
                    MAT_ELEMENT(*V,j,i) = ((MAT_ELEMENT(*U,i,j) / MAT_ELEMENT(*U,i,l)) / g);
                    /* float division to avoid underflow */
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < n; k++)
                        s += (MAT_ELEMENT(*U,i,k) * MAT_ELEMENT(*V,k,j));
                    for (k = l; k < n; k++)
                        MAT_ELEMENT(*V,k,j) += (s * MAT_ELEMENT(*V,k,i));
                }
            }
            for (j = l; j < n; j++)
                MAT_ELEMENT(*V,i,j) = MAT_ELEMENT(*V,j,i) = 0.0;
        }
        MAT_ELEMENT(*V,i,i) = 1.0;
        g = rv1[i];
        l = i;
    }

    /* accumulate the left-hand transformation */
    for (i = n - 1; i >= 0; i--)
    {
        l = i + 1;
        g = MAT_ELEMENT(*S,i,i);
        if (i < n - 1)
            for (j = l; j < n; j++)
                MAT_ELEMENT(*U,i,j) = 0.0;
        if (g)
        {
            g = 1.0 / g;
            if (i != n - 1)
            {
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < m; k++)
                        s += (MAT_ELEMENT(*U,k,i) * MAT_ELEMENT(*U,k,j));
                    f = (s / MAT_ELEMENT(*U,i,i)) * g;
                    for (k = i; k < m; k++)
                        MAT_ELEMENT(*U,k,j) += (f * MAT_ELEMENT(*U,k,i));
                }
            }
            for (j = i; j < m; j++)
                MAT_ELEMENT(*U,j,i) = (MAT_ELEMENT(*U,j,i)*g);
        }
        else
        {
            for (j = i; j < m; j++)
                MAT_ELEMENT(*U,j,i) = 0.0;
        }
        ++MAT_ELEMENT(*U,i,i);
    }

    /* diagonalize the bidiagonal form */
    for (k = n - 1; k >= 0; k--)
    {                             /* loop over singular values */
        for (its = 0; its < 30; its++)
        {                         /* loop over allowed iterations */
            flag = 1;
            for (l = k; l >= 0; l--)
            {                     /* test for splitting */
                nm = l - 1;
                if (fabsf(rv1[l]) + anorm == anorm)
                {
                    flag = 0;
                    break;
                }
                if (fabsf(MAT_ELEMENT(*S,nm,nm)) + anorm == anorm)
                    break;
            }
            if (flag)
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++)
                {
                    f = s * rv1[i];
                    if (fabsf(f) + anorm != anorm)
                    {
                        g = MAT_ELEMENT(*S,i,i);
                        h = PYTHAGF(f, g);
                        MAT_ELEMENT(*S,i,i) = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = (- f * h);
                        for (j = 0; j < m; j++)
                        {
                            y = MAT_ELEMENT(*U,j,nm);
                            z = MAT_ELEMENT(*U,j,i);
                            MAT_ELEMENT(*U,j,nm) = (y * c + z * s);
                            MAT_ELEMENT(*U,j,i) = (z * c - y * s);
                        }
                    }
                }
            }
            z = MAT_ELEMENT(*S,k,k);
            if (l == k)
            {                  /* convergence */
                if (z < 0.0)
                {              /* make singular value nonnegative */
                    MAT_ELEMENT(*S,k,k) = (-z);
                    for (j = 0; j < n; j++)
                        MAT_ELEMENT(*V,j,k) = (-MAT_ELEMENT(*V,j,k));
                }
                break;
            }

            if (its >= 30)
                return ARM_MATH_ARGUMENT_ERROR; // No convergence after 30,000! iterations

            /* shift from bottom 2 x 2 minor */
            x = MAT_ELEMENT(*S,l,l);
            nm = k - 1;
            y = MAT_ELEMENT(*S,nm,nm);
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = PYTHAGF(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGNF(g, f))) - h)) / x;

            /* next QR transformation */
            c = s = 1.0;
            for (j = l; j <= nm; j++)
            {
                i = j + 1;
                g = rv1[i];
                y = MAT_ELEMENT(*S,i,i);
                h = s * g;
                g = c * g;
                z = PYTHAGF(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;
                for (jj = 0; jj < n; jj++)
                {
                    x = MAT_ELEMENT(*V,jj,j);
                    z = MAT_ELEMENT(*V,jj,i);
                    MAT_ELEMENT(*V,jj,j) = (x * c + z * s);
                    MAT_ELEMENT(*V,jj,i) = (z * c - x * s);
                }
                z = PYTHAGF(f, h);
                MAT_ELEMENT(*S,j,j) = z;
                if (z)
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = (c * g) + (s * y);
                x = (c * y) - (s * g);
                for (jj = 0; jj < m; jj++)
                {
                    y = MAT_ELEMENT(*U,jj,j);
                    z = MAT_ELEMENT(*U,jj,i);
                    MAT_ELEMENT(*U,jj,j) = (y * c + z * s);
                    MAT_ELEMENT(*U,jj,i) = (z * c - y * s);
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            MAT_ELEMENT(*S,k,k) = x;
        }
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_pinv(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
	// stack usage 4*10*10 = 400B
	// maximum matrix size is SVD_MAX_SIZE * SVD_MAX_SIZE
	uint8_t transposed = 0;
	arm_matrix_instance_f32 U = {A->numRows, A->numCols, (float[SVD_MAX_SIZE*SVD_MAX_SIZE]){}};

	if(A->numRows < A->numCols)
	{
		U.numRows = A->numCols;
		U.numCols = A->numRows;
		arm_mat_trans_f32(A, &U);
		transposed = 1;
	}
	else
	{
		U.numRows = A->numRows;
		U.numCols = A->numCols;
		memcpy(U.pData, A->pData, sizeof(float)*A->numRows*A->numCols);
	}

	arm_matrix_instance_f32 S = {U.numCols, U.numCols, (float[SVD_MAX_SIZE*SVD_MAX_SIZE]){}};
	arm_matrix_instance_f32 V = {U.numCols, U.numCols, (float[SVD_MAX_SIZE*SVD_MAX_SIZE]){}};
	arm_matrix_instance_f32 tmp1 = {U.numCols, U.numRows, (float[SVD_MAX_SIZE*SVD_MAX_SIZE]){}};

	arm_status svdResult = arm_mat_svd(&U, &S, &V);
	if(svdResult)
		return svdResult;

	// Moore-Penrose pseudo-inverse
	// >>> V * inv(S) * U'

	// S = inv(S)
	for(uint16_t i = 0; i < S.numCols; i++)
	{
		if(MAT_ELEMENT(S, i, i) != 0.0f)
			MAT_ELEMENT(S, i, i) = 1.0f/MAT_ELEMENT(S, i, i);
	}

	// tmp1 = U'
	arm_mat_trans_f32(&U, &tmp1);

	// U = S*tmp1
	float tmp = U.numCols;
	U.numCols = U.numRows;
	U.numRows = tmp;
	arm_mat_mult_f32(&S, &tmp1, &U);

	// tmp1 = V*U
	tmp1.numRows = U.numRows;
	tmp1.numCols = U.numCols;
	arm_mat_mult_f32(&V, &U, &tmp1);

	if(transposed)
		arm_mat_trans_f32(&tmp1, B);
	else
		memcpy(B->pData, tmp1.pData, sizeof(float)*A->numRows*A->numCols);

	return ARM_MATH_SUCCESS;
}

void CalibIMU_Init()
{
	LagElementPT1Init(&lagAccel[0], 1.0f, 0.01f, CTRL_DELTA_T);
	LagElementPT1Init(&lagAccel[1], 1.0f, 0.01f, CTRL_DELTA_T);
}

void CalibIMU()
	{

	float Gyro_xyz[3] = {0};
	float gyro_x = 0.0;
	float gyro_y = 0.0;
	float gyro_z = 0.0;

	int16_t Accel_xyz[3] = {0};
	float accel_x = 0.0;
	float accel_y = 0.0;
	float accel_z = 0.0;

	float acc = 0.0;
	float g = 0.0;

	int first_time = 1;

	float accBias[3];

	uint32_t last_time = 0;
	uint32_t actual_time = 0;
	float dt = 0.0f;

	c=p;
//	initialise_monitor_handles();
	fp= fopen("C:\\Users\\joaov\\accel_wrong.txt", "w");

	printf("Getting wrong data from accelerometer");
	printf("\n");

	samples_view = 0;

	for(uint16_t sample_accel_wrong = 0; sample_accel_wrong < Accel_Get_Data_Wrong; sample_accel_wrong++)
	{
		samples_view = sample_accel_wrong;

		BSP_ACCELERO_GetXYZ(Accel_xyz);

		accel_x = Accel_xyz[0] * 0.061f / 0.10197162129779f / 1000.f - 0.0181531906 - 0.0407385752;// - 0.0398350358; //+ 0.0498709083 - 0.0869560316; //- 0.0427414179 + 0.0343631543;//- 0.0156089067 - 0.00662654266;// - 0.00135274674;//- 0.0580787659 + 0.0433597006;//+ 0.0498709083 - 0.0869560316; //- 0.0638287961; //- 0.0872829258;//
		//					accel_x = LagElementPT1Process(&lagAccel[0], accel_x);
		fprintf(fp, "%.6f", accel_x);
		fprintf(fp, ";");

		accel_y = Accel_xyz[1] * 0.061f / 0.10197162129779f / 1000.f + 0.529428422 - 0.13063921;// + 0.486646235;// + 0.379035175 - 0.00845457055;//+ 0.307901144 + 0.0691580102;// + 0.0528672934;//+ 0.385120869 - 0.0395160019;//+ 0.266912103 + 0.144043803; //+ 0.121962726; //+ 0.0696646869;// + 0.0879048631;//
		//					accel_y = LagElementPT1Process(&lagAccel[1], accel_y);
		fprintf(fp, "%.6f", accel_y);
		fprintf(fp, ";");

		accel_z = Accel_xyz[2] * 0.061f / 0.10197162129779f / 1000.f + 0.425669312;// + 0.58314842;// + 0.452083707;// + 0.42904371;//+ 0.405499727 ;//+ 0.452083707;
		//					accel_z = LagElementPT1Process(&lagAccel[0], accel_z);
		fprintf(fp, "%.6f", accel_z);
		fprintf(fp, ";");

		acc = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
		fprintf(fp, "%.6f",acc);
		fprintf(fp, ";");

//		if(sample_accel_wrong == 99){
//			samples_view = sample_accel_wrong;
//		}

		BSP_GYRO_GetXYZ(Gyro_xyz);

		gyro_x = (Gyro_xyz[0] / 1000.0);
		fprintf(fp, "%.6f",gyro_x);
		fprintf(fp, ";");

		gyro_y = (Gyro_xyz[1] / 1000.0);
		fprintf(fp, "%.6f",gyro_y);
		fprintf(fp, ";");

		gyro_z = (Gyro_xyz[2] / 1000.0);
		fprintf(fp, "%.6f",gyro_z);
		fprintf(fp, ";");
		fprintf(fp, "\n");
	}

	fclose(fp);

	printf("Starting IMU Calibration");
	printf("\n");

	static double records[IMU_CALIB_NUM_SAMPLES][6];
		memset(records, 0, sizeof(records));

		SignalStatistics stats[6] = { 0 };

		const uint32_t numSamples = 5000;

		for(uint16_t sample = 0; sample < IMU_CALIB_NUM_SAMPLES; sample++)
			{
				repeat_view = sample;
				for(uint8_t i = 0; i < 6; i++)
				{
					SignalStatisticsReset(&stats[i]);
				}

				samples_view = 0;

				for(uint32_t t = 0; t < numSamples; t++)
				{
					samples_view = t;

//					actual_time = HAL_GetTick();
//					dt = (actual_time - last_time);
//					last_time = actual_time;

					BSP_ACCELERO_GetXYZ(Accel_xyz);

					accel_x = Accel_xyz[0] * 0.061f / 0.10197162129779f / 1000.f;
					accel_y = Accel_xyz[1] * 0.061f / 0.10197162129779f / 1000.f;
					accel_z = Accel_xyz[2] * 0.061f / 0.10197162129779f / 1000.f;

					BSP_GYRO_GetXYZ(Gyro_xyz);
					gyro_x = (Gyro_xyz[0] / 1000.0);
					gyro_y = (Gyro_xyz[1] / 1000.0);
					gyro_z = (Gyro_xyz[2] / 1000.0);

					SignalStatisticsSample(&stats[0], gyro_x);
					SignalStatisticsSample(&stats[1], gyro_y);
					SignalStatisticsSample(&stats[2], gyro_z);
					SignalStatisticsSample(&stats[3], accel_x);
					SignalStatisticsSample(&stats[4], accel_y);
					SignalStatisticsSample(&stats[5], accel_z);
				}

				for(uint8_t i = 0; i < 6; i++)
				{
					SignalStatisticsUpdate(&stats[i]);
				}

				for(uint8_t i = 0; i < 6; i++)
				{
					printf("%d", i);
					printf("\n");
					printf("Variance: ");
					printf("%.6f", stats[i].variance);
					printf("\n");
					printf("Mean: ");
					printf("%.6f", stats[i].mean);
					printf("\n");
					printf("Std Deviation: ");
					printf("%.6f", stats[i].standardDeviation);
					printf("\n");
					records[sample][i] = stats[i].mean;
				}
			}

		// Compute acceleromter bias

		// Determine sphere from acceleration points (center == bias, magnitude should be ~9.81)
		//    (ax+bx)^2 + (ay+by)^2 + (az+bz)^2 = g^2
		// => bx^2 + by^2 + bz^2 - g^2 - 2*ax*bx - 2*ay*by - 2*az*bz = -(ax^2 + ay^2 + az^2)
		// set: P = bx^2 + by^2 + bz^2 - g^2, Q = -2*bx, R = -2*by, S = -2*bz
		// => P + Q*ax + R*ay + S*az = -(ax^2 + ay^2 + az^2)
		// vector x = [P Q R S]'
		// matrix A = [1 ax ay az; ..]
		// vector b = [-(ax^2 + ay^2 + az^2); ..]

		float32_t matAccA[IMU_CALIB_NUM_SAMPLES][4] = { 0 };
		float32_t matAccb[IMU_CALIB_NUM_SAMPLES][1] = { 0 };
		arm_matrix_instance_f32 mat_A_instance;
		arm_matrix_instance_f32 mat_B_instance;

	    arm_mat_init_f32(&mat_A_instance, IMU_CALIB_NUM_SAMPLES, 4, &matAccA[0][0]);
	    arm_mat_init_f32(&mat_B_instance, IMU_CALIB_NUM_SAMPLES, 1, &matAccb[0][0]);

	    fp= fopen("C:\\Users\\joaov\\sphere_data.txt", "w");

		for(uint16_t sample = 0; sample < IMU_CALIB_NUM_SAMPLES; sample++)
		{
			matAccA[sample][0] = 1.0f;
			matAccA[sample][1] = records[sample][3];
			matAccA[sample][2] = records[sample][4];
			matAccA[sample][3] = records[sample][5];

			matAccb[sample][0] = -(records[sample][3]*records[sample][3] +
					records[sample][4]*records[sample][4] +
					records[sample][5]*records[sample][5]);

			fprintf(fp, "%.6f", records[sample][3]);
			fprintf(fp, ";");
			fprintf(fp, "%.6f", records[sample][4]);
			fprintf(fp, ";");
			fprintf(fp, "%.6f", records[sample][5]);
			fprintf(fp, ";");
		}

		fclose(fp);

		float32_t matAccAInv[4][IMU_CALIB_NUM_SAMPLES] = { 0 };
		arm_matrix_instance_f32 mat_A_Inv_instance;

		arm_mat_init_f32(&mat_A_Inv_instance, 4, IMU_CALIB_NUM_SAMPLES, &matAccAInv[0][0]);
		arm_mat_pinv(&mat_A_instance, &mat_A_Inv_instance);

		float32_t matAccx[4][1] = { 0 };
		arm_matrix_instance_f32 mat_X_instance;

		arm_mat_init_f32(&mat_X_instance, 4, 1, &matAccx[0][0]);
		arm_mat_mult_f32(&mat_A_Inv_instance, &mat_B_instance, &mat_X_instance);

		accBias[0] = -matAccx[1][0] * 0.5f;
		accBias[1] = -matAccx[2][0] * 0.5f;
		accBias[2] = -matAccx[3][0] * 0.5f;

		float g2 = accBias[0]*accBias[0] + accBias[1]*accBias[1] + accBias[2]*accBias[2] - matAccx[0][0];
		g = sqrtf(fabsf(g2));

		// Compute gyro bias (simple average)
		float gyroBias[3] = { 0 };

		for(uint16_t sample = 0; sample < IMU_CALIB_NUM_SAMPLES; sample++)
		{
			gyroBias[0] += records[sample][0];
			gyroBias[1] += records[sample][1];
			gyroBias[2] += records[sample][2];
		}

		gyroBias[0] /= (float)IMU_CALIB_NUM_SAMPLES;
		gyroBias[1] /= (float)IMU_CALIB_NUM_SAMPLES;
		gyroBias[2] /= (float)IMU_CALIB_NUM_SAMPLES;

		memset(records, 0, sizeof(records));
		float gyroIntegrationZ = 0.0f;

		const uint32_t numSamplesTilt = 3000;

		#define IMU_CALIB_NUM_ROT_SAMPLES 4

		for(uint16_t sample = 0; sample < IMU_CALIB_NUM_ROT_SAMPLES; sample++)
		{

			for(uint8_t i = 0; i < 6; i++)
			{
				SignalStatisticsReset(&stats[i]);
			}

			printf("Turn the robot 90º");

			for(uint8_t i = 0; i < 1000; i++)
			{
				BSP_GYRO_GetXYZ(Gyro_xyz);
				gyro_z = (Gyro_xyz[2] / 1000) - gyroBias[2];//+ 0.522795022;// + 0.631300;

				gyroIntegrationZ += gyro_z * dt;
			}

			printf("Getting data to compute accel tilt");

			for(uint32_t t = 0; t < numSamplesTilt; t++)
			{
				actual_time = HAL_GetTick();
				dt = (actual_time - last_time)*1e-3;

				if(first_time == 1)
				{
					dt = 0;
					first_time = 0;
				}

				last_time = actual_time;
				BSP_ACCELERO_GetXYZ(Accel_xyz);
				accel_x = Accel_xyz[0] * 0.061f / 0.10197162129779f / 1000.f - accBias[0];
				accel_y = Accel_xyz[1] * 0.061f / 0.10197162129779f / 1000.f - accBias[1];
				accel_z = Accel_xyz[2] * 0.061f / 0.10197162129779f / 1000.f - accBias[2];

				BSP_GYRO_GetXYZ(Gyro_xyz);
				gyro_z = (Gyro_xyz[2] / 1000) - gyroBias[2];

				gyroIntegrationZ += gyro_z * dt;

				SignalStatisticsSample(&stats[3], accel_x);
				SignalStatisticsSample(&stats[4], accel_y);
				SignalStatisticsSample(&stats[5], accel_z);
			}

			for(uint8_t i = 3; i < 6; i++)
			{
				SignalStatisticsUpdate(&stats[i]);
			}

			records[sample][0] = gyroIntegrationZ;

			for(uint8_t i = 3; i < 6; i++)
			{
				records[sample][i] = stats[i].mean;
			}
		}

		float32_t matTiltA[IMU_CALIB_NUM_ROT_SAMPLES*2][4] = { 0 };
		float32_t matTiltb[IMU_CALIB_NUM_ROT_SAMPLES*2][1] = { 0 };
		arm_matrix_instance_f32 mat_tiltA_instance;
		arm_matrix_instance_f32 mat_tiltB_instance;

		arm_mat_init_f32(&mat_tiltA_instance, IMU_CALIB_NUM_ROT_SAMPLES*2, 4, &matTiltA[0][0]);
		arm_mat_init_f32(&mat_tiltB_instance, IMU_CALIB_NUM_ROT_SAMPLES*2, 1, &matTiltb[0][0]);

		for(uint16_t sample = 0; sample < IMU_CALIB_NUM_ROT_SAMPLES; sample++)
		{
			MAT_ELEMENT(mat_tiltA_instance, sample*2+0, 0) = cosf(records[sample][0]);
			MAT_ELEMENT(mat_tiltA_instance, sample*2+0, 1) = sinf(records[sample][0]);
			MAT_ELEMENT(mat_tiltA_instance, sample*2+0, 2) = 1.0f;
			MAT_ELEMENT(mat_tiltA_instance, sample*2+0, 3) = 0.0f;
			MAT_ELEMENT(mat_tiltA_instance, sample*2+1, 0) = -sinf(records[sample][0]);
			MAT_ELEMENT(mat_tiltA_instance, sample*2+1, 1) = cosf(records[sample][0]);
			MAT_ELEMENT(mat_tiltA_instance, sample*2+1, 2) = 0.0f;
			MAT_ELEMENT(mat_tiltA_instance, sample*2+1, 3) = 1.0f;

			MAT_ELEMENT(mat_tiltB_instance, sample*2+0, 0) = records[sample][3];
			MAT_ELEMENT(mat_tiltB_instance, sample*2+1, 0) = records[sample][4];
		}

		float32_t matTiltAInv[4][IMU_CALIB_NUM_ROT_SAMPLES*2] = { 0 };
		arm_matrix_instance_f32 mat_tiltA_Inv_instance;

		arm_mat_init_f32(&mat_tiltA_Inv_instance, 4, IMU_CALIB_NUM_ROT_SAMPLES*2, &matTiltAInv[0][0]);
		arm_mat_pinv(&mat_tiltA_instance, &mat_tiltA_Inv_instance);

		float32_t matTiltx[4][1] = { 0 };
		arm_matrix_instance_f32 mat_tiltX_instance;

		arm_mat_init_f32(&mat_tiltX_instance, 4, 1, &matTiltx[0][0]);
		arm_mat_mult_f32(&mat_tiltA_Inv_instance, &mat_tiltB_instance, &mat_tiltX_instance);

		float accTiltBias[2];
		accTiltBias[0] = MAT_ELEMENT(mat_tiltX_instance, 2, 0);
		accTiltBias[1] = MAT_ELEMENT(mat_tiltX_instance, 3, 0);

		printf("Gyro Bias 0: %.8f, Gyro Bias 1: %.8f, Gyro Bias 2: %.8f", gyroBias[0], gyroBias[1], gyroBias[2]);
		printf("\n");
		printf("Accel Bias 0: %.8f, Accel Bias 1: %.8f, Accel Bias 2: %.8f", accBias[0], accBias[1], accBias[2]);
		printf("\n");
		printf("Tilt Bias 0: %.8f, Tilt Bias 1: %.8f", accTiltBias[0], accTiltBias[1]);
		printf("\n");
}

static float PYTHAGF(float a, float b)
{
	float at = fabsf(a), bt = fabsf(b), ct, result;

    if (at > bt)       { ct = bt / at; result = at * sqrtf(1.0f + ct * ct); }
    else if (bt > 0.0f) { ct = at / bt; result = bt * sqrtf(1.0f + ct * ct); }
    else result = 0.0f;
    return(result);
}




