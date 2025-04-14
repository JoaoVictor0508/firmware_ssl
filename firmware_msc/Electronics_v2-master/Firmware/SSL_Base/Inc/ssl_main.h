/*
 * ssl_main.h
 *
 *  Created on: 26 de ago de 2022
 *      Author: leonardo
 */

#ifndef INC_SSL_MAIN_H_
#define INC_SSL_MAIN_H_

#define ARM_MATH_CM4
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "kickConfig.h"
#include "main.h"
#include "motorControl.h"
#include "nRF24L01.h"

#define LOOP_uS 5000           // Tempo do loop principal [us]
#define BATTERY_READ_TIMEOUT 2 // Timeout da leitura da bateria [ms]
#define MIN_BATTERY 11000      // Bateria mínima para operação [mV]
#define DEBUG_SIZE 32          // Quantidade de bytes de debug enviados
#define BALL_SENSOR_FILTER_ON                                                  \
    5 // Espera o sensor ficar ligado por A ciclos
       // (A x LOOP_uS) para considerar como ativado
#define BALL_SENSOR_FILTER_OFF                                                 \
    30 // Espera o sensor ficar desligado por A ciclos
       // (A x LOOP_uS) para considerar como ativado
#define KICK_CYCLES                                                            \
    40 // Mantém o comando de chute por 40 ciclos (200 ms)
       // antes de desligar

#define __GET_HIGH_BYTE(__VALUE__) ((__VALUE__ & 0xFF00) >> 8)
#define __GET_LOW_BYTE(__VALUE__) (__VALUE__ & 0x00FF)

#define CAPACITOR_SAMPLES 20
static uint8_t capacitorVoltages[CAPACITOR_SAMPLES] = {0};
static uint16_t capacitorIndex = 0;

typedef struct _RobotSensors
{
	struct
	{
		uint32_t updated;
		uint32_t time;
		float rotVel[3];	// around X, Y, Z [rad/s]
		float lastRotVel[3];
		float accel_angular;
	} gyr;

	struct
	{
		uint32_t updated;
		uint32_t time;
		float linAcc[3];	// X, Y, Z [m^2/s]
		float lastLinAcc[3];
	} acc;

	struct
	{
		uint32_t updated;
		uint32_t time;
		float localVel[3];
	} encoder;

	struct
	{
		uint8_t updated;
		uint32_t time;
		float theoVel[3];
	} theoreticalVel;

	struct PACKED
	{
		uint32_t updated;
		uint32_t time;		// local arrival time (systime)
		uint32_t delay;		// t [us]
		float pos[3];		// X, Y, theta [m]
//		uint32_t camId;
//		uint32_t noVision;
	} vision;
} RobotSensors;

typedef struct _RobotMath
{
	float theta_rad[4];	// wheel angles in [rad]

	arm_matrix_instance_f32 matXYW2Motor; // 4x3
	arm_matrix_instance_f32 matMotor2XYW; // 3x4
} RobotMath;

typedef struct _DriveTrainParams
{
	float motor2WheelRatio;
	float wheel2MotorRatio;
//	MotorParams motor;
} DriveTrainParams;

typedef struct _PhysicalParams
{
	float wheelRadius_m;
	float frontAngle_deg;
	float backAngle_deg;
	float botRadius_m; // Robot center to wheel/ground contact point
	float mass_kg;
	float dribblerDistance_m; // from center of robot to center of ball in front of robot
	float dribblerWidth_m; // width the ball can move while at the dribbler
	float centerOfGravity_m[3]; // measured from geometric robot center point on ground
	float massDistributionZ; // Factor between 0.5 (mass evenly distributed/solid cylinder) and 1.0 (all mass at outer radius/hollow shell), used for inertia calculation
	float gyroDistToCenter_m; //Distance of the gyroscope (U7 element in the border schematic) to the robot center to compensate the angular velocity
	float accelDistToCenter_m; //Distance of the gyroscope (U5 element in the border schematic) to the robot center to compensate the acceleration
} PhysicalParams;

typedef struct _RobotSpecs
{
	DriveTrainParams driveTrain;
//	DribblerParams dribbler;
	PhysicalParams physical;
} RobotSpecs;

typedef struct _RobotState
{
	float pos[3];	// [m]
	float vel[3];	// [m/s]
	float Sigma[5][5];
	float kalman_gain[5][3];

	float global_vel[2];
	float global_accel[2];

	float theta_int_gyro;
	float theta_int_enc;

	float pos_enc[3];
	float vel_enc[3];
	float pos_accel[3];
	float vel_accel[3];

	uint8_t imu_counter;
	uint8_t enc_counter;
} RobotState;

typedef struct RobotData_t
{
    uint32_t battery;
    int16_t mVbattery;

    float offsetAcc[3];
    float offsetGyr[3];

    RobotSensors sensors;

    float pos_imu[3];
    float vel_imu[3];

    RobotSpecs specs;

    RobotMath math;

    RobotState state;

    uint32_t estimateTime;

    uint8_t capacitorVoltage;
    int16_t wheelSpeed[MOTOR_SZ];
    int16_t encoderCount[MOTOR_SZ];
    bool ballSensor;
    uint32_t ballCycles;

    uint32_t radioCycles;

    uint32_t cycles;

    uint32_t lastTime;

    uint8_t deltaT;

    KICK_TYPE_t kickType;
    int8_t kickStrength;
    uint16_t kickControl;
    uint32_t kickCyles;
    bool debugKick;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t theta;
    } pose;
    bool rollerEnable;
    int8_t rollerSpeed; // Velocidade do roller em rad/s
    uint32_t rollerTimer;

} RobotData_t;

//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t kickSensor;      // [0]
//        uint8_t robotID;         // [1]
//        uint8_t battery;         // [2]
//        uint8_t sensorType;      // [3]
//        uint8_t wheelSpeed1High; // [4]
//        uint8_t wheelSpeed1Low;  // [5]
//        uint8_t wheelSpeed2High; // [6]
//        uint8_t wheelSpeed2Low;  // [7]
//        uint8_t wheelSpeed3High; // [8]
//        uint8_t wheelSpeed3Low;  // [9]
//        uint8_t wheelSpeed4High; // [10]
//        uint8_t wheelSpeed4Low;  // [11]
//        uint8_t posXHigh;      // [12]
//        uint8_t posXLow;       // [13]
//        uint8_t posYHigh;      // [14]
//        uint8_t posYLow;       // [15]
//        uint8_t posThetaHigh;      // [16]
//        uint8_t posThetaLow;       // [17]
//        uint8_t velXHigh;      // [18]
//        uint8_t velXLow;       // [19]
//        uint8_t velYHigh;          // [20]
//        uint8_t velYLow;          // [21]
//        uint8_t capacitorVoltage;// [22]
//        uint8_t packetFrequency; // [23]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posVisionXHigh;   	// [0]
//        uint8_t posVisionXLow;   	// [1]
//        uint8_t posVisionYHigh;  	// [2]
//        uint8_t posVisionYLow;   	// [3]
//        uint8_t posVisionThetaHigh; // [4]
//        uint8_t posVisionThetaLow;  // [5]
//        uint8_t posXHigh;      		// [6]
//        uint8_t posXLow;       		// [7]
//        uint8_t posYHigh;      		// [8]
//        uint8_t posYLow;       		// [9]
//        uint8_t posThetaHigh;       // [10]
//        uint8_t posThetaLow;    	// [11]
//        uint8_t velXHigh;       	// [12]
//        uint8_t velXLow;        	// [13]
//        uint8_t velYHigh;       	// [14]
//        uint8_t velYLow;        	// [15]
//		uint8_t velThetaGyroHigh;	// [16]
//		uint8_t velThetaGyroLow;	// [17]
//		uint8_t velThetaEncHigh;	// [18]
//		uint8_t velThetaEncLow;		// [19]
////        uint8_t accelXHigh;			// [20]
////        uint8_t accelXLow;			// [21]
////        uint8_t accelYHigh;			// [22]
////		uint8_t accelYLow;			// [23]
////        uint8_t velXTheoHigh; // [16]
////		uint8_t velXTheoLow; // [17]
////		uint8_t velYTheoHigh; // [18]
////		uint8_t velYTheoLow; // [19]
////		uint8_t velWTheoHigh; // [20]
////		uint8_t velWTheoLow; // [21]
//		uint8_t velXEncoderHigh; //[20]
//		uint8_t velXEncoderLow; // [21]
//		uint8_t velYEncoderHigh; //[22]
//		uint8_t velYEncoderLow; // [23]
//
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

// Debug data para testar cenário IMU + Visão no Python

//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posVisionXHigh;   	// [0]
//        uint8_t posVisionXLow;   	// [1]
//        uint8_t posVisionYHigh;  	// [2]
//        uint8_t posVisionYLow;   	// [3]
//        uint8_t posVisionThetaHigh; // [4]
//        uint8_t posVisionThetaLow;  // [5]
//        uint8_t posXHigh;      		// [6]
//        uint8_t posXLow;       		// [7]
//        uint8_t posYHigh;      		// [8]
//        uint8_t posYLow;       		// [9]
//        uint8_t posThetaHigh;       // [10]
//        uint8_t posThetaLow;    	// [11]
//        uint8_t accelXHigh;			// [12]
//        uint8_t accelXLow;			// [13]
//        uint8_t accelYHigh;			// [14]
//        uint8_t accelYLow;			// [15]
//		uint8_t velThetaGyroHigh;	// [16]
//		uint8_t velThetaGyroLow;	// [17]
//		uint8_t velXHigh; 			// [18]
//		uint8_t velXLow; 			// [19]
//		uint8_t velYHigh; 			// [20]
//		uint8_t velYLow; 			// [21]
//		uint8_t deltaT;				// [22]
//
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//Debug data para o cenario modelo + encoder e modelo + visao
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;   	// [0]
//        uint8_t posXLow;   	// [1]
//        uint8_t posYHigh;  	// [2]
//        uint8_t posYLow;   	// [3]
//        uint8_t posThetaHigh; // [4]
//        uint8_t posThetaLow;  // [5]
//        uint8_t velXHigh;      		// [6]
//        uint8_t velXLow;       		// [7]
//        uint8_t velYHigh;      		// [8]
//        uint8_t velYLow;       		// [9]
//        uint8_t velTheoXHigh;		// [10]
//		uint8_t velTheoXLow;		// [11]
//        uint8_t velTheoYHigh;		// [12]
//        uint8_t velTheoYLow;		// [13]
//        uint8_t velTheoWHigh;		// [14]
//        uint8_t velTheoWLow;		// [15]
//		uint8_t velEncXHigh;		// [16]
//		uint8_t velEncXLow;			// [17]
//		uint8_t velEncYHigh; 		// [18]
//		uint8_t velEncYLow; 		// [19]
//		uint8_t velEncWHigh; 		// [20]
//		uint8_t velEncWLow; 		// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//Debug data para o cenario modelo + encoder
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;      		// [0]
//        uint8_t posXLow;       		// [1]
//        uint8_t posYHigh;      		// [2]
//        uint8_t posYLow;       		// [3]
//        uint8_t posThetaHigh;
//        uint8_t posThetaLow;
//        uint8_t velXHigh;
//        uint8_t velXLow;
//        uint8_t velYHigh;
//        uint8_t velYLow;
//        uint8_t velTheoXHigh;		// [10]
//		uint8_t velTheoXLow;		// [11]
//        uint8_t velTheoYHigh;		// [12]
//        uint8_t velTheoYLow;		// [13]
//        uint8_t velTheoWHigh;		// [14]
//        uint8_t velTheoWLow;		// [15]
//		uint8_t velEncXHigh;		// [16]
//		uint8_t velEncXLow;			// [17]
//		uint8_t velEncYHigh; 		// [18]
//		uint8_t velEncYLow; 		// [19]
//		uint8_t velEncWHigh; 		// [20]
//		uint8_t velEncWLow; 		// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//Debug data para o cenario modelo + imu
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;      		// [0]
//        uint8_t posXLow;       		// [1]
//        uint8_t posYHigh;      		// [2]
//        uint8_t posYLow;       		// [3]
//        uint8_t posThetaHigh;		// [4]
//        uint8_t posThetaLow;		// [5]
//        uint8_t velXHigh;			// [6]
//        uint8_t velXLow;			// [7]
//        uint8_t velYHigh;			// [8]
//        uint8_t velYLow;			// [9]
//        uint8_t velTheoXHigh;		// [10]
//		uint8_t velTheoXLow;		// [11]
//        uint8_t velTheoYHigh;		// [12]
//        uint8_t velTheoYLow;		// [13]
//        uint8_t velTheoWHigh;		// [14]
//        uint8_t velTheoWLow;		// [15]
//		uint8_t accelXHigh;			// [16]
//		uint8_t accelXLow;			// [17]
//		uint8_t accelYHigh; 		// [18]
//		uint8_t accelYLow; 			// [19]
//		uint8_t gyroWHigh; 			// [20]
//		uint8_t gyroWLow; 			// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

// Debug data para o cenario modelo + IMU
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;   		// [0]
//        uint8_t posXLow;   			// [1]
//        uint8_t posYHigh;  			// [2]
//        uint8_t posYLow;   			// [3]
//        uint8_t posThetaHigh; 		// [4]
//        uint8_t posThetaLow;  		// [5]
//        uint8_t velXHigh;      		// [6]
//        uint8_t velXLow;       		// [7]
//        uint8_t velYHigh;      		// [8]
//        uint8_t velYLow;       		// [9]
//		uint8_t posXAccelHigh;		// [10]
//		uint8_t posXAccelLow;			// [11]
//		uint8_t posYAccelHigh; 		// [12]
//		uint8_t posYAccelLow; 		// [13]
//		uint8_t posThetaAccelHigh; 		// [14]
//		uint8_t posThetaAccelLow; 		// [15]
//		uint8_t velXAccelHigh;			// [16]
//		uint8_t velXAccelLow;			// [17]
//		uint8_t velYAccelHigh; 		// [18]
//		uint8_t velYAccelLow; 			// [19]
//		uint8_t gyroWHigh; 			// [20]
//		uint8_t gyroWLow; 			// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

////Debug data para o cenario encoder + visao
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;   		// [0]
//        uint8_t posXLow;   			// [1]
//        uint8_t posYHigh;  			// [2]
//        uint8_t posYLow;   			// [3]
//        uint8_t posThetaHigh; 		// [4]
//        uint8_t posThetaLow;  		// [5]
//        uint8_t velXHigh;      		// [6]
//        uint8_t velXLow;       		// [7]
//        uint8_t velYHigh;      		// [8]
//        uint8_t velYLow;       		// [9]
//		uint8_t velEncXHigh;		// [10]
//		uint8_t velEncXLow;			// [11]
//		uint8_t velEncYHigh; 		// [12]
//		uint8_t velEncYLow; 		// [13]
//		uint8_t velEncWHigh; 		// [14]
//		uint8_t velEncWLow; 		// [15]
//		uint8_t data16;				// [16]
//		uint8_t data17;				// [17]
//		uint8_t data18; 			// [18]
//		uint8_t data19; 			// [19]
//		uint8_t data20; 			// [20]
//		uint8_t data21; 			// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//Debug data para o cenario imu + visao
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;   		// [0]
//        uint8_t posXLow;   			// [1]
//        uint8_t posYHigh;  			// [2]
//        uint8_t posYLow;   			// [3]
//        uint8_t posThetaHigh; 		// [4]
//        uint8_t posThetaLow;  		// [5]
//        uint8_t velXHigh;      		// [6]
//        uint8_t velXLow;       		// [7]
//        uint8_t velYHigh;      		// [8]
//        uint8_t velYLow;       		// [9]
//		uint8_t velEncXHigh;		// [10]
//		uint8_t velEncXLow;			// [11]
//		uint8_t velEncYHigh; 		// [12]
//		uint8_t velEncYLow; 		// [13]
//		uint8_t velEncWHigh; 		// [14]
//		uint8_t velEncWLow; 		// [15]
//		uint8_t accelXHigh;			// [16]
//		uint8_t accelXLow;			// [17]
//		uint8_t accelYHigh; 		// [18]
//		uint8_t accelYLow; 			// [19]
//		uint8_t gyroWHigh; 			// [20]
//		uint8_t gyroWLow; 			// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//Debug data para o cenario encoder + imu
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;   		// [0]
//        uint8_t posXLow;   			// [1]
//        uint8_t posYHigh;  			// [2]
//        uint8_t posYLow;   			// [3]
//        uint8_t posThetaHigh; 		// [4]
//        uint8_t posThetaLow;  		// [5]
//        uint8_t velXHigh;      		// [6]
//        uint8_t velXLow;       		// [7]
//        uint8_t velYHigh;      		// [8]
//        uint8_t velYLow;       		// [9]
//		uint8_t velEncXHigh;		// [10]
//		uint8_t velEncXLow;			// [11]
//		uint8_t velEncYHigh; 		// [12]
//		uint8_t velEncYLow; 		// [13]
//		uint8_t velEncWHigh; 		// [14]
//		uint8_t velEncWLow; 		// [15]
//		uint8_t accelXHigh;			// [16]
//		uint8_t accelXLow;			// [17]
//		uint8_t accelYHigh; 		// [18]
//		uint8_t accelYLow; 			// [19]
//		uint8_t gyroWHigh; 			// [20]
//		uint8_t gyroWLow; 			// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//Debug data para o cenario encoder + imu
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;   		// [0]
//        uint8_t posXLow;   			// [1]
//        uint8_t posYHigh;  			// [2]
//        uint8_t posYLow;   			// [3]
//        uint8_t posThetaHigh; 		// [4]
//        uint8_t posThetaLow;  		// [5]
//        uint8_t velXHigh;      		// [6]
//        uint8_t velXLow;       		// [7]
//        uint8_t velYHigh;      		// [8]
//        uint8_t velYLow;       		// [9]
//		uint8_t posXEncHigh;		// [10]
//		uint8_t posXEncLow;			// [11]
//		uint8_t posYEncHigh; 		// [12]
//		uint8_t posYEncLow; 		// [13]
//		uint8_t posThetaEncHigh; 		// [14]
//		uint8_t posThetaEncLow; 		// [15]
//		uint8_t velXEncHigh;			// [16]
//		uint8_t velXEncLow;			// [17]
//		uint8_t velYEncHigh; 		// [18]
//		uint8_t velYEncLow; 			// [19]
//		uint8_t gyroWHigh; 			// [20]
//		uint8_t gyroWLow; 			// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//Debug data para o cenario imu + encoder
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;      		// [0]
//        uint8_t posXLow;       		// [1]
//        uint8_t posYHigh;      		// [2]
//        uint8_t posYLow;       		// [3]
//        uint8_t posThetaHigh;		// [4]
//        uint8_t posThetaLow;		// [5]
//        uint8_t velXHigh;			// [6]
//        uint8_t velXLow;			// [7]
//        uint8_t velYHigh;			// [8]
//        uint8_t velYLow;			// [9]
//		uint8_t velEncXHigh;		// [10]
//		uint8_t velEncXLow;			// [11]
//		uint8_t velEncYHigh; 		// [12]
//		uint8_t velEncYLow; 		// [13]
//		uint8_t velEncWHigh; 		// [14]
//		uint8_t velEncWLow; 		// [15]
//		uint8_t accelXHigh;			// [16]
//		uint8_t accelXLow;			// [17]
//		uint8_t accelYHigh; 		// [18]
//		uint8_t accelYLow; 			// [19]
//		uint8_t gyroWHigh; 			// [20]
//		uint8_t gyroWLow; 			// [21]
//		uint8_t deltaT;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

// DebugData para testes finais de posição e tempo
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;      		// [0]
//        uint8_t posXLow;       		// [1]
//        uint8_t posYHigh;      		// [2]
//        uint8_t posYLow;       		// [3]
//        uint8_t posThetaHigh;		// [4]
//        uint8_t posThetaLow;		// [5]
//        uint8_t velXHigh;			// [6]
//        uint8_t velXLow;			// [7]
//        uint8_t velYHigh;			// [8]
//        uint8_t velYLow;			// [9]
//        uint8_t posXAccelHigh;		// [10]
//		uint8_t posXAccelLow;		// [11]
//        uint8_t posYAccelHigh;		// [12]
//        uint8_t posYAccelLow;		// [13]
//        uint8_t posWAccelHigh;		// [14]
//        uint8_t posWAccelLow;		// [15]
//		uint8_t velXAccelHigh;		// [16]
//		uint8_t velXAccelLow;			// [17]
//		uint8_t velYAccelHigh; 		// [18]
//		uint8_t velYAccelLow; 			// [19]
//		uint8_t deltaTHigh; 			// [20]
//		uint8_t deltaTLow; 			// [21]
//		uint8_t data22;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

// DebugData para testes finais de velocidade
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;      		// [0]
//        uint8_t posXLow;       		// [1]
//        uint8_t velXHigh;			// [2]
//        uint8_t velXLow;			// [3]
//        uint8_t velYHigh;			// [4]
//        uint8_t velYLow;			// [5]
//		uint8_t velXAccelHigh;		// [6]
//		uint8_t velXAccelLow;		// [7]
//		uint8_t velYAccelHigh; 		// [8]
//		uint8_t velYAccelLow; 		// [9]
//        uint8_t velXTheoHigh;		// [10]
//		uint8_t velXTheoLow;		// [11]
//        uint8_t velYTheoHigh;		// [12]
//        uint8_t velYTheoLow;		// [13]
//        uint8_t velXEncHigh;		// [14]
//        uint8_t velXEncLow;			// [15]
//		uint8_t velYEncHigh;		// [16]
//		uint8_t velYEncLow;			// [17]
//		uint8_t data18; 			// [18]
//		uint8_t data19; 			// [19]
//		uint8_t data20; 			// [20]
//		uint8_t data21; 			// [21]
//		uint8_t data22;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//debugData para testes finais de encoder e IMU
//typedef union DebugData_t {
//    struct
//    {
//        //      informação ---- [indice]
//        uint8_t posXHigh;   		// [0]
//        uint8_t posXLow;   			// [1]
//        uint8_t posYHigh;  			// [2]
//        uint8_t posYLow;   			// [3]
//        uint8_t posThetaHigh; 		// [4]
//        uint8_t posThetaLow;  		// [5]
//        uint8_t posXVisionHigh;     // [6]
//        uint8_t posXVisionLow;      // [7]
//        uint8_t posYVisionHigh;     // [8]
//        uint8_t posYVisionLow;      // [9]
//		uint8_t posThetaVisionHigh;		// [10]
//		uint8_t posThetaVisionLow;				// [11]
//		uint8_t velXHigh; 			// [12]
//		uint8_t velXLow;		 		// [13]
//		uint8_t velYHigh;		 		// [14]
//		uint8_t velYLow; 		// [15]
//		uint8_t velThetaHigh;				// [16]
//		uint8_t velThetaLow;				// [17]
//		uint8_t velYEncHigh; 			// [18]
//		uint8_t velYEncLow; 			// [19]
//		uint8_t data20; 			// [20]
//		uint8_t data21; 			// [21]
//		uint8_t flagAttPose;				// [22]
//    };
//    uint8_t data[DEBUG_SIZE];
//} DebugData_t;

//DEBUG DA INTEGRAL DA IMU
typedef union DebugData_t {
    struct
    {
        //      informação ---- [indice]
        uint8_t posXHigh;   		// [0]
        uint8_t posXLow;   			// [1]
        uint8_t posYHigh;  			// [2]
        uint8_t posYLow;   			// [3]
        uint8_t posThetaHigh; 		// [4]
        uint8_t posThetaLow;  		// [5]
        uint8_t velXIMUHigh;     	// [6]
        uint8_t velXIMULow;      	// [7]
        uint8_t velYIMUHigh;     // [8]
        uint8_t velYIMULow;      // [9]
		uint8_t velThetaIMUHigh;		// [10]
		uint8_t velThetaIMULow;				// [11]
		uint8_t velXEncHigh; 			// [12]
		uint8_t velXEncLow;		 		// [13]
		uint8_t velYEncHigh;		 		// [14]
		uint8_t velYEncLow; 		// [15]
		uint8_t velThetaEncHigh;				// [16]
		uint8_t velThetaEncLow;				// [17]
		uint8_t posYEKFHigh; 			// [18]
		uint8_t posYEKFLow; 			// [19]
		uint8_t data20; 			// [20]
		uint8_t data21; 			// [21]
		uint8_t flagAttPose;				// [22]
    };
    uint8_t data[DEBUG_SIZE];
} DebugData_t;

static RobotData_t robotData = {.battery = 0,
                                .mVbattery = 0,
                                .capacitorVoltage = 0,
                                .wheelSpeed = {0, 0, 0, 0, 0},
                                .encoderCount = {0, 0, 0, 0, 0},
                                .ballSensor = false,
                                .ballCycles = 0,
								.sensors.gyr.rotVel = {0.0, 0.0, 0.0},
								.sensors.acc.linAcc = {0, 0, 0},
                                .radioCycles = 0,
                                .cycles = 0,
                                .kickType = KICK_NONE,
                                .kickStrength = 0,
                                .kickControl = CYCLES_BETWEEN_KICKS,
                                .kickCyles = KICK_CYCLES,
                                .debugKick = false,
                                .pose = {.x = 0, .y = 0, .theta = 0},
                                .rollerEnable = false,
								.specs.physical.wheelRadius_m = 27.0*1e-3,
								.specs.physical.gyroDistToCenter_m = 16*1e-3,
								.specs.physical.accelDistToCenter_m = 10.2*1e-3,
								.specs.driveTrain.motor2WheelRatio = 20.0 / 60.0,
								.specs.driveTrain.wheel2MotorRatio = 60.0 / 20.0,
								.specs.physical.frontAngle_deg = 33.0,
								.specs.physical.backAngle_deg = 33.0,
								.specs.physical.botRadius_m = 80.5*1e-3,
                                .rollerTimer = 0};

static DebugData_t debugData;

/**
 * @brief Faz a configuração inicial e inicialização do firmware
 */
void setup();

/**
 * @brief Roda o controle dos motores
 */
void runMotors();

void HAL_IncTick(void);

/**
 * @brief Retorna o ID atual do robô [0, 15]
 *
 * @return
 */
uint8_t getRobotID();

/**
 * @brief Mostra o ID do robô
 *
 * Atualmente isso é feito piscando um led na quantidade de vezes igual ao ID
 */
void showRobotID();

/**
 * @brief Lê o canal atual do rádio
 *
 * @param _rx - Valor do canal de recebimento de dados. f = 2400 + _rx MHz.
 * @param _tx - Valor do canal de envio de dados. f = 2400 + _tx MHz.
 */
void getRobotChannel(uint8_t* _rx, uint8_t* _tx);

/**
 * @brief A cada 200 ciclos (1s) checa se o ID ou canal mudou
 */
void checkRobotIDChannel();

/**
 * @brief Processa um pacote do rádio caso algum tenha chegado
 *
 * @param _sensorData
 */
void processRadio();

/**
 * @brief A cada 200 ciclos (1s) lê o valor da bateria
 */
void readBattery();

/**
 * @brief Atualiza os dados enviados de volta para a estratégia
 */
void updateDebugInfo();

void convertDebugSpeed(uint8_t* _high, uint8_t* _low, int16_t _speed);

/**
 * @brief Se estiver ativado, faz o robô chutar
 */
void setKickCommand();

/**
 * @brief Lê o sensor da bola
 */
void readBallSensor();

/**
 * @brief Faz a estimativa de posição e velocidade do robô
 */
void estimateState();

void motorVelToLocalVel(const int16_t* pMotor, float* pLocal);

#endif /* INC_SSL_MAIN_H_ */
