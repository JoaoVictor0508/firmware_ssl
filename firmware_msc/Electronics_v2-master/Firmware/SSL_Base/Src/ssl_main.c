/*
 * ssl_main.c
 *
 *  Created on: 26 de ago de 2022
 *      Author: leonardo
 */

#include "ssl_main.h"
#include "main.h"
#include "motorControl.h"
#include "pid.h"
#include "pwm.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f411e_discovery_accelerometer.h"
#include "stm32f411e_discovery_gyroscope.h"
#include "ekf_fusion.h"
#include "l3gd20.h"
#include "lsm303dlhc.h"
#include "lag_element.h"
#include "signal_statistics.h"
#include <stdio.h>

#include "encoder.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc1;

extern TIM_HandleTypeDef htim11;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;

extern uint32_t delayCount;

SPI_HandleTypeDef* radioSPI = &hspi3;

TIM_HandleTypeDef* microSecTimer = &htim11; // Channel N/A

TIM_HandleTypeDef* pwmMotor1 = &htim10;     // TIM10 CH 1
TIM_HandleTypeDef* pwmMotor2 = &htim9;      // TIM9  CH 2
TIM_HandleTypeDef* pwmMotor3 = &htim5;      // TIM5  CH 2
TIM_HandleTypeDef* pwmMotor4 = &htim5;      // TIM5  CH 4
TIM_HandleTypeDef* pwmMotorRoller = &htim5; // TIM5  CH 3

TIM_HandleTypeDef* encoderM1 = &htim3; // TIM3 - CH1 - CH2
TIM_HandleTypeDef* encoderM2 = &htim2; // TIM2 - CH1 - CH2
TIM_HandleTypeDef* encoderM3 = &htim1; // TIM1 - CH1 - CH2
TIM_HandleTypeDef* encoderM4 = &htim4; // TIM4 - CH1 - CH2
volatile uint32_t loopCount = 0, loopTime = 0;

extern __IO uint16_t uhADCxConvertedValue[2];

RadioRXPacket_t receivedPacket;
SPIData_t radioData;

#define SVD_MAX_SIZE 10

#define SIGNF(a, b) ((b) >= 0.0 ? fabsf(a) : -fabsf(a))
#define MAX(x,y) ((x)>(y)?(x):(y))
static float PYTHAGF(float a, float b);

#define CTRL_DELTA_T_IN_US 4000
#define CTRL_DELTA_T (CTRL_DELTA_T_IN_US*1e-6f)

#define CTRL_MOTOR_TO_WHEEL_RATIO robotData.specs.driveTrain.motor2WheelRatio
#define CTRL_WHEEL_TO_MOTOR_RATIO robotData.specs.driveTrain.wheel2MotorRatio

arm_status arm_mat_pinv(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);
arm_status arm_mat_svd(arm_matrix_instance_f32* U, arm_matrix_instance_f32* S, arm_matrix_instance_f32* V);

void calibrateSensors();
void updateRobotMath();

void RobotMathMotorVelToLocalVel(const int16_t* pMotor, float* pLocal);

LagElementPT1 lagAccel[2];
LagElementPT1 lagGyro[1];

static FusionEKFConfig configFusionKF = {
	.posNoiseXY = 0.0001f,
	.posNoiseW = 0.0001f,
	.velNoiseXY = 0.0005f,
	.visNoiseXY = 0.01f,
	.visNoiseW = 0.01f,
	.visNoiseVel = 0.00001f,
//	.visNoiseXY = 0.001f,
//	.visNoiseW = 0.01f,
	.outlierMaxVelXY = 3.0f,
	.outlierMaxVelW = 3.0f,
	.emaAccelT = 0.005f,
};

extern void initialise_monitor_handles(void);

int main(void)
{
    setup();

    bool kickTest = false;

    while (true)
    {
        loopCount = __HAL_TIM_GET_COUNTER(microSecTimer);

        if (pwmBuzzer.play)
        {
            playSong();
        }

        if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) ==
            GPIO_PIN_SET)
        {
            if (kickTest)
            {
                robotData.kickType = DIRECT_CUSTOM;
            }
            else
            {
                robotData.kickType = CHIP_STRONG;
            }
            kickTest = !kickTest;
            robotData.kickStrength = MAX_KICK_STRENGTH;
            robotData.debugKick = true;
        }

        readBattery();

        if (robotData.mVbattery > MIN_BATTERY)
        {
            processRadio();
            estimateState();

            runMotors();
            setKickCommand();
            checkRobotIDChannel();
        }
        else
        {
            // Faz a kick parar de carregar quando a bateria fica baixa
            // HAL_GPIO_WritePin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin,
            // GPIO_PIN_SET);
            int8_t v = 0;
            setMotorsCommand(&v, &v, &v, false);
            runMotorControl();
        }

        loopTime = max(__HAL_TIM_GET_COUNTER(microSecTimer), loopCount) -
                   min(__HAL_TIM_GET_COUNTER(microSecTimer), loopCount);
        if (loopTime < LOOP_uS)
        {
            Delay_us(LOOP_uS - loopTime);
        }
        robotData.rollerTimer += LOOP_uS;
        robotData.cycles++;
    }
    return 0;
}

void setup()
{
    init();
    HAL_GPIO_WritePin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHOOT_EN_GPIO_Port, SHOOT_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CHIP_KICK_GPIO_Port, CHIP_KICK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_KICK_GPIO_Port, RIGHT_KICK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEFT_KICK_GPIO_Port, LEFT_KICK_Pin, GPIO_PIN_RESET);

//    initialise_monitor_handles();

    HAL_Delay(500);

    radioData.robotID = getRobotID();
    getRobotChannel(&radioData.rfRXChannel, &radioData.rfTXChannel);

    FusionEKFInit(&configFusionKF);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();

    calibrateSensors();
    updateRobotMath();

	LagElementPT1Init(&lagAccel[0], 1.0f, 0.01f, CTRL_DELTA_T);
	LagElementPT1Init(&lagAccel[1], 1.0f, 0.01f, CTRL_DELTA_T);
	LagElementPT1Init(&lagGyro[0], 1.0f, 0.01f, CTRL_DELTA_T);

    nRF24_RadioConfig();
    showRobotID();
    initMotors();
    initEncoders();
    initPWM();
    readBattery();

    for (uint8_t id = MOTOR_1; id < MOTOR_SZ - 1; ++id)
    {
        setGains(&pidControl[id], 50, 10, 0);
    }
}

void runMotors()
{
    updateEncoders();
    getSpeed(&robotData.wheelSpeed[MOTOR_1], &robotData.wheelSpeed[MOTOR_2],
             &robotData.wheelSpeed[MOTOR_3], &robotData.wheelSpeed[MOTOR_4]);
    getCounts(
        &robotData.encoderCount[MOTOR_1], &robotData.encoderCount[MOTOR_2],
        &robotData.encoderCount[MOTOR_3], &robotData.encoderCount[MOTOR_4]);
    updateFeedback(
        &robotData.wheelSpeed[MOTOR_1], &robotData.wheelSpeed[MOTOR_2],
        &robotData.wheelSpeed[MOTOR_3], &robotData.wheelSpeed[MOTOR_4]);

    if (robotData.rollerEnable == true && robotData.ballSensor == true)
    {
        if (robotData.rollerSpeed <= 100 && robotData.rollerTimer > 20000)
        {
            robotData.rollerTimer = 0;
            robotData.rollerSpeed += 1;
        }
    }
    else if (robotData.rollerEnable && !robotData.ballSensor)
    {
        robotData.rollerSpeed = 10;
    }
    setRollerCommand(&robotData.rollerSpeed);

    runMotorControl(); // Controle via PWM
}

void calibrateSensors()
{
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_SET);

	uint16_t calibration_samples = 600;

	float Gyro_xyz[3] = {0};
	float gyro_x = 0.0;
	float gyro_y = 0.0;
	float gyro_z = 0.0;

	float gyro_x_validation = 0.0;
	float gyro_y_validation = 0.0;
	float gyro_z_validation = 0.0;

	int16_t Accel_xyz[3] = {0};
	float accel_x = 0.0;
	float accel_y = 0.0;
	float accel_z = 0.0;

	float accel_x_validation = 0.0;
	float accel_y_validation = 0.0;
	float accel_z_validation = 0.0;

	bool calibrated = false;

	while(calibrated == false)
	{
		for(uint16_t sample = 0; sample < calibration_samples; sample++)
		{
			BSP_ACCELERO_GetXYZ(Accel_xyz);

			accel_x += Accel_xyz[0] * 0.061f / 0.10197162129779f / 1000.f;
			accel_y += Accel_xyz[1] * 0.061f / 0.10197162129779f / 1000.f;
			accel_z += Accel_xyz[2] * 0.061f / 0.10197162129779f / 1000.f;

			BSP_GYRO_GetXYZ(Gyro_xyz);

			gyro_x += (Gyro_xyz[0] / 1000.0);
			gyro_y += (Gyro_xyz[1] / 1000.0);
			gyro_z += (Gyro_xyz[2] / 1000.0);
		}

		robotData.offsetAcc[0] = accel_x / calibration_samples;
		robotData.offsetAcc[1] = accel_y / calibration_samples;
		robotData.offsetAcc[2] = 9.81 - accel_z / calibration_samples;

		robotData.offsetGyr[0] = gyro_x / calibration_samples;
		robotData.offsetGyr[1] = gyro_y / calibration_samples;
		robotData.offsetGyr[2] = gyro_z / calibration_samples;

		uint16_t validation_samples = 50;

		for(uint16_t sample = 0; sample < 50; sample++)
		{
			BSP_ACCELERO_GetXYZ(Accel_xyz);

			accel_x_validation += Accel_xyz[0] * 0.061f / 0.10197162129779f / 1000.f - robotData.offsetAcc[0];
			accel_y_validation += Accel_xyz[1] * 0.061f / 0.10197162129779f / 1000.f - robotData.offsetAcc[1];
			accel_z_validation += Accel_xyz[2] * 0.061f / 0.10197162129779f / 1000.f + robotData.offsetAcc[2];

			BSP_GYRO_GetXYZ(Gyro_xyz);

			gyro_x_validation += (Gyro_xyz[0] / 1000.0) - robotData.offsetGyr[0];
			gyro_y_validation += (Gyro_xyz[1] / 1000.0) - robotData.offsetGyr[1];
			gyro_z_validation += (Gyro_xyz[2] / 1000.0) - robotData.offsetGyr[2];
		}
//
//		printf("Gyro X: ");
//		printf("%.6f", gyro_x_validation / validation_samples);
//		printf("\n");
//		printf("Gyro Y: ");
//		printf("%.6f", gyro_y_validation / validation_samples);
//		printf("\n");
//		printf("Gyro Z: ");
//		printf("%.6f", gyro_z_validation / validation_samples);
//		printf("\n");
//		printf("Accel X: ");
//		printf("%.6f", accel_x_validation / validation_samples);
//		printf("\n");
//		printf("Accel Y: ");
//		printf("%.6f", accel_y_validation / validation_samples);
//		printf("\n");
//		printf("Accel Z: ");
//		printf("%.6f", accel_z_validation / validation_samples);
//		printf("\n");

		calibrated = true;

		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
	}
}

void updateRobotMath()
{
	static float pXYW2Motor[12];
	static float pMotor2XYW[12];

	robotData.math.matXYW2Motor = (arm_matrix_instance_f32){4, 3, pXYW2Motor};
	robotData.math.matMotor2XYW = (arm_matrix_instance_f32){3, 4, pMotor2XYW};

	float frontAngle_deg = robotData.specs.physical.frontAngle_deg;
	float backAngle_deg = robotData.specs.physical.backAngle_deg;
	float botRadius_m = robotData.specs.physical.botRadius_m;

	float alpha_rad = frontAngle_deg *(M_PI)/180.0f;
	float beta_rad = backAngle_deg * (M_PI)/180.0f;

	robotData.math.theta_rad[0] = M_PI-alpha_rad;
	robotData.math.theta_rad[1] = M_PI+beta_rad;
	robotData.math.theta_rad[2] = 2*M_PI-beta_rad;
	robotData.math.theta_rad[3] = alpha_rad;

	for(uint8_t i = 0; i < 4; i++)
	{
		// construct matrix for XYW velocity to motor velocity conversion
		MAT_ELEMENT(robotData.math.matXYW2Motor, i, 0) = -sinf(robotData.math.theta_rad[i]);
		MAT_ELEMENT(robotData.math.matXYW2Motor, i, 1) = cosf(robotData.math.theta_rad[i]);
		MAT_ELEMENT(robotData.math.matXYW2Motor, i, 2) = botRadius_m;
	}

	float validation[4][3];

	for(int k = 0; k < 4; k++)
	{
		for(int a = 0; a < 3; a++)
		{
			validation[k][a] = MAT_ELEMENT(robotData.math.matXYW2Motor, k, a);
		}
	}

	arm_mat_pinv(&robotData.math.matXYW2Motor, &robotData.math.matMotor2XYW);
}

void HAL_IncTick()
{
    uwTick += uwTickFreq;
    // runMotors();
    // updateEncoders();
    // updateFeedback(
    //     &robotData.wheelSpeed[MOTOR_1], &robotData.wheelSpeed[MOTOR_2],
    //     &robotData.wheelSpeed[MOTOR_3], &robotData.wheelSpeed[MOTOR_4]);
    // runMotorControl(); // Controle via PWM
}

uint8_t getRobotID()
{
    uint8_t id = 0;
    id = HAL_GPIO_ReadPin(ROBOT_ID_0_GPIO_Port, ROBOT_ID_0_Pin) * 1;
    id += HAL_GPIO_ReadPin(ROBOT_ID_1_GPIO_Port, ROBOT_ID_1_Pin) * 2;
    id += HAL_GPIO_ReadPin(ROBOT_ID_2_GPIO_Port, ROBOT_ID_2_Pin) * 4;
    id += HAL_GPIO_ReadPin(ROBOT_ID_3_GPIO_Port, ROBOT_ID_3_Pin) * 8;
    return id;
}

void showRobotID()
{
    // HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
    for (int i = 0; i < 2 * radioData.robotID; ++i)
    {
        HAL_GPIO_TogglePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin);
        HAL_Delay(300);
    }

    if (radioData.robotID == 0)
    {
        HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
    }
    HAL_Delay(200);
    HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
}

void getRobotChannel(uint8_t* _rx, uint8_t* _tx)
{
    if (HAL_GPIO_ReadPin(ROBOT_CH_GPIO_Port, ROBOT_CH_Pin) == GPIO_PIN_SET)
    {
        *_rx = _NRF24L01P_CHANNEL_A_RX;
        *_tx = _NRF24L01P_CHANNEL_A_TX;
    }
    else
    {
        *_rx = _NRF24L01P_CHANNEL_B_RX;
        *_tx = _NRF24L01P_CHANNEL_B_TX;
    }
}

void checkRobotIDChannel()
{
    if (robotData.cycles % 200 == 0)
    {
        bool configChanged = false;
        if (getRobotID() != radioData.robotID)
        {
            radioData.robotID = getRobotID();
            configChanged = true;
        }

        uint8_t rx, tx;
        getRobotChannel(&rx, &tx);
        if (rx != radioData.rfRXChannel || tx != radioData.rfTXChannel)
        {
            radioData.rfRXChannel = rx;
            radioData.rfTXChannel = tx;
            configChanged = true;
        }

        if (configChanged)
        {
            showRobotID();
            nRF24_RadioConfig();
        }
    }
}

void processRadio()
{
    if (radioData.dataReceived == true)
    {
        robotData.radioCycles = robotData.cycles;
        nRF24_ReadData();
        setMotorsCommand(&receivedPacket.velocityX, &receivedPacket.velocityY,
                         &receivedPacket.velocityW, true);

        if (receivedPacket.rollerVelocity > 0)
        {
            robotData.rollerEnable = true;
            if (robotData.rollerSpeed < 10)
            {
                robotData.rollerSpeed = 10;
            }
        }
        else
        {
            robotData.rollerEnable = false;
            robotData.rollerSpeed = 0;
            setRollerCommand(&robotData.rollerSpeed);
        }

        if (robotData.kickControl == 0)
        {
            robotData.kickType = receivedPacket.kickType;
            robotData.kickStrength = receivedPacket.kickStrength;
            // robotData.kickCyles = KICK_CYCLES;
        }
        else if (receivedPacket.kickType == KICK_NONE)
        {
            robotData.kickType = receivedPacket.kickType;
            robotData.kickStrength = 0;
        }

        if (robotData.pose.x != receivedPacket.x.value || robotData.pose.y != receivedPacket.y.value || robotData.pose.theta != receivedPacket.theta.value)
        {
        	robotData.pose.x = receivedPacket.x.value;
			robotData.pose.y = receivedPacket.y.value;
			robotData.pose.theta = receivedPacket.theta.value;

			robotData.sensors.vision.updated = 1;
        }
        else
        {
        	robotData.sensors.vision.updated = 0;
        }
    }
    else if (robotData.cycles - robotData.radioCycles >= 500)
    {
        int8_t v = 0;
        setMotorsCommand(&v, &v, &v, false);
        runMotorControl();
        // nRF24_RadioConfig();
        NVIC_SystemReset();
        robotData.radioCycles = robotData.cycles;
    }

    if (receivedPacket.sendFeedback == true)
    {
        receivedPacket.sendFeedback = false;
        updateDebugInfo();
        nRF24_Transmit(debugData.data);
    }
}

void readBattery()
{

    if (robotData.cycles % 20 == 0)
    {
        robotData.capacitorVoltage = 0;
        uint64_t acc = 0;
        for (int i = 0; i < CAPACITOR_SAMPLES; i++)
        {
            acc += capacitorVoltages[i];
        }
        robotData.capacitorVoltage = acc / CAPACITOR_SAMPLES;
        // HAL_ADC_Start_IT(&hadc1);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, 2);
    }
}

void updateDebugInfo()
{
    debugData.kickSensor = robotData.ballSensor;
    debugData.robotID = radioData.robotID;
    debugData.battery = robotData.battery; // 89 ~ 11.95V
    debugData.sensorType = 0;

    convertDebugSpeed(&debugData.wheelSpeed1High, &debugData.wheelSpeed1Low,
                      robotData.wheelSpeed[MOTOR_1]);
    convertDebugSpeed(&debugData.wheelSpeed2High, &debugData.wheelSpeed2Low,
                      robotData.wheelSpeed[MOTOR_2]);
//    convertDebugSpeed(&debugData.wheelSpeed3High, &debugData.wheelSpeed3Low,
//                      robotData.wheelSpeed[MOTOR_3]);
//    convertDebugSpeed(&debugData.wheelSpeed4High, &debugData.wheelSpeed4Low,
//                      robotData.wheelSpeed[MOTOR_4]);

    convertDebugSpeed(&debugData.wheelSpeed3High, &debugData.wheelSpeed3Low,
    					(int)(robotData.sensors.acc.linAcc[0]*1000.0));
    convertDebugSpeed(&debugData.wheelSpeed4High, &debugData.wheelSpeed4Low,
    					(int)(robotData.sensors.acc.linAcc[1]*1000.0));

//    convertDebugSpeed(&debugData.count1High, &debugData.count1Low,
//                      robotData.encoderCount[MOTOR_1]);
//    convertDebugSpeed(&debugData.count2High, &debugData.count2Low,
//                      robotData.encoderCount[MOTOR_2]);
//    convertDebugSpeed(&debugData.count3High, &debugData.count3Low,
//                      robotData.encoderCount[MOTOR_3]);
//    convertDebugSpeed(&debugData.count4High, &debugData.count4Low,
//                      robotData.encoderCount[MOTOR_4]);

    convertDebugSpeed(&debugData.posXHigh, &debugData.posXLow,
    					(int)(robotData.state.pos[0]*1000.0));
    convertDebugSpeed(&debugData.posYHigh, &debugData.posYLow,
    					(int)(robotData.state.pos[1]*1000.0));

//    convertDebugSpeed(&debugData.posXHigh, &debugData.posXLow,
//    					(int)(robotData.sensors.acc.linAcc[0]*1000.0));
//    convertDebugSpeed(&debugData.posYHigh, &debugData.posYLow,
//    					(int)(robotData.sensors.acc.linAcc[1]*1000.0));

    convertDebugSpeed(&debugData.posThetaHigh, &debugData.posThetaLow,
    					(int)(robotData.state.pos[2]));

//    convertDebugSpeed(&debugData.velXHigh, &debugData.velXLow,
//					  	(int)(robotData.state.vel[0]*1000.0));
//    convertDebugSpeed(&debugData.velYHigh, &debugData.velYLow,
//    					(int)(robotData.state.vel[1]*1000.0));

	convertDebugSpeed(&debugData.velXHigh, &debugData.velXLow,
						(int)(robotData.sensors.gyr.rotVel[2])); //debug da velocidade de rotação com giroscópio
	convertDebugSpeed(&debugData.velYHigh, &debugData.velYLow,
						(int)(robotData.sensors.encoder.localVel[2] * 180.0 / M_PI)); //debug da velocidade de rotação com encoder
    debugData.capacitorVoltage = robotData.capacitorVoltage;
    debugData.packetFrequency = radioData.packetFrequency;
}

void convertDebugSpeed(uint8_t* _high, uint8_t* _low, int16_t _speed)
{
    int8_t high, low;
    if (_speed >= 0)
    {
        high = __GET_HIGH_BYTE(_speed);
        low = __GET_LOW_BYTE(_speed);
    }
    else
    {
        _speed *= -1;
        high = __GET_HIGH_BYTE(_speed);
        high &= 0x0F;
        high |= 0x10;
        low = __GET_LOW_BYTE(_speed);
    }
    *_high = high;
    *_low = low;
}

void setKickCommand()
{
    if (robotData.kickControl > 0)
    {
        robotData.kickControl--;
    }
    else
    {
        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
    }

    // if(robotData.kickCyles > 0)
    // {
    //     robotData.kickCyles--;
    // }
    // else
    // {
    //     robotData.kickType = KICK_NONE;
    //     robotData.kickStrength = 0;
    // }

    readBallSensor();

    if (robotData.kickType != KICK_NONE && robotData.kickControl == 0 &&
        (robotData.ballSensor == true || robotData.debugKick == true))
    {
        HAL_GPIO_WritePin(SHOOT_EN_GPIO_Port, SHOOT_EN_Pin, GPIO_PIN_RESET);
        // HAL_Delay(2); // Espera o pino estabilizar

        float kickDelay = 0;
        // GPIO_TypeDef *kickPort = RIGHT_KICK_GPIO_Port;
        // uint16_t kickPin = RIGHT_KICK_Pin | LEFT_KICK_Pin;
        GPIO_TypeDef* kickPort = CHIP_KICK_GPIO_Port;
        uint16_t kickPin = CHIP_KICK_Pin;

        // Port e pino
        if (robotData.kickType == DIRECT_SOFT ||
            robotData.kickType == DIRECT_CUSTOM ||
            robotData.kickType == DIRECT_STRONG)
        {
            // O port deve ser o mesmo para o código funcionar
            // Caso contrário o acionamento muda
            assert_param(RIGHT_KICK_GPIO_Port == LEFT_KICK_GPIO_Port);
            HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
            kickPort = RIGHT_KICK_GPIO_Port;
            kickPin = RIGHT_KICK_Pin;
        }
        else if (robotData.kickType == CHIP_SOFT ||
                 robotData.kickType == CHIP_CUSTOM ||
                 robotData.kickType == CHIP_STRONG)
        {
            HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
            kickPort = CHIP_KICK_GPIO_Port;
            kickPin = CHIP_KICK_Pin;
        }
        else if (robotData.kickType == ANGLE_KICK)
        {
            assert_param(RIGHT_KICK_GPIO_Port == LEFT_KICK_GPIO_Port);
            HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
            kickPort = RIGHT_KICK_GPIO_Port;
            kickPin = RIGHT_KICK_Pin;
            kickPin = RIGHT_KICK_Pin | LEFT_KICK_Pin;
        }

        // Delay
        if (robotData.kickType == DIRECT_SOFT ||
            robotData.kickType == CHIP_SOFT)
        {
            kickDelay = MIN_KICK_ACTIVATION;
        }
        else if (robotData.kickType == DIRECT_STRONG ||
                 robotData.kickType == CHIP_STRONG)
        {
            kickDelay = MAX_KICK_ACTIVATION;
        }
        else
        {
            kickDelay = min(robotData.kickStrength, MAX_KICK_STRENGTH) *
                        MAX_KICK_ACTIVATION / MAX_KICK_STRENGTH;
        }

        if (robotData.kickStrength != 0)
        {
            kickDelay = min(robotData.kickStrength, MAX_KICK_STRENGTH) *
                        MAX_KICK_ACTIVATION / MAX_KICK_STRENGTH;
        }
        // kickDelay = MAX_KICK_ACTIVATION;

        // Acionamento
        if (robotData.kickType != ANGLE_KICK)
        {
            HAL_GPIO_WritePin(kickPort, kickPin, GPIO_PIN_SET);
            Delay_us(kickDelay * 1000);
            HAL_GPIO_WritePin(kickPort, kickPin, GPIO_PIN_RESET);
        }
        else
        {
            // TODO Fazer o acionamento diferencial das bobinas
        }

        HAL_GPIO_WritePin(SHOOT_EN_GPIO_Port, SHOOT_EN_Pin, GPIO_PIN_SET);
        robotData.kickControl = CYCLES_BETWEEN_KICKS;
        robotData.kickStrength = 0;
        robotData.kickType = KICK_NONE;
    }
}

void readBallSensor()
{
    if (HAL_GPIO_ReadPin(BALL_SENSOR_GPIO_Port, BALL_SENSOR_Pin) ==
        GPIO_PIN_SET)
    {
        if (robotData.cycles - robotData.ballCycles > BALL_SENSOR_FILTER_OFF)
        {
            robotData.ballSensor = false;
            robotData.ballCycles = robotData.cycles;
        }
    }
    else
    {
        if (robotData.cycles - robotData.ballCycles > BALL_SENSOR_FILTER_ON)
        {
            robotData.ballSensor = true;
            robotData.ballCycles = robotData.cycles;
        }
    }
}

void estimateState()
{
	float gyro_xyz[3] = {0};
	int16_t accel_xyz[3] = {0};

    BSP_ACCELERO_GetXYZ(accel_xyz);
	robotData.sensors.acc.linAcc[0] = LagElementPT1Process(&lagAccel[1], accel_xyz[1] * 0.061f / 0.10197162129779f / 1000.f - robotData.offsetAcc[1]); // m/s^2
	robotData.sensors.acc.linAcc[1] = LagElementPT1Process(&lagAccel[0], accel_xyz[0] * 0.061f / 0.10197162129779f / 1000.f - robotData.offsetAcc[0]); // m/s^2
	robotData.sensors.acc.linAcc[2] = accel_xyz[2] * 0.061f / 0.10197162129779f / 1000.f + robotData.offsetAcc[2]; // m/s^2

	BSP_GYRO_GetXYZ(gyro_xyz);
	robotData.sensors.gyr.rotVel[0] = (gyro_xyz[0] / 1000) - robotData.offsetGyr[0]; // º/s
	robotData.sensors.gyr.rotVel[1] = (gyro_xyz[1] / 1000) - robotData.offsetGyr[1]; // º/s
	robotData.sensors.gyr.rotVel[2] = LagElementPT1Process(&lagGyro[0],((gyro_xyz[2] / 1000) - robotData.offsetGyr[2]) / (1 + (robotData.specs.physical.gyroDistToCenter_m / robotData.specs.physical.botRadius_m))); // º/s

	robotData.sensors.vision.pos[0] = robotData.pose.x / 1000.0; // m
	robotData.sensors.vision.pos[1] = robotData.pose.y / 1000.0; // m
	robotData.sensors.vision.pos[2] = robotData.pose.theta * M_PI / 180.0; // rad

	robotData.sensors.theoreticalVel.theoVel[0] = receivedPacket.velocityX * MAX_VELOCITY / 100.0; // m/s
	robotData.sensors.theoreticalVel.theoVel[1] = receivedPacket.velocityY * MAX_VELOCITY / 100.0; // m/s
	robotData.sensors.theoreticalVel.theoVel[2] = receivedPacket.velocityW * MAX_ROTATION / 100.0; // rad/s

	float encoderVel[3];

	RobotMathMotorVelToLocalVel(robotData.wheelSpeed, encoderVel); // wheel speed in rpm, converted to m/s

	robotData.sensors.encoder.localVel[0] = -encoderVel[1];
	robotData.sensors.encoder.localVel[1] = encoderVel[0];
	robotData.sensors.encoder.localVel[2] = encoderVel[2];

//	FusionEKFUpdate(&robotData.sensors, &robotData.state);
//	FusionEKFUpdate_encoder_vision(&robotData.sensors, &robotData.state);
	FusionEKFUpdate_encoder_imu(&robotData.sensors, &robotData.state);
//	FusionEKFUpdate_imu_encoder(&robotData.sensors, &robotData.state);
}

void RobotMathMotorVelToLocalVel(const int16_t* pMotor, float* pLocal)
{
	float motor[4] = {pMotor[0], pMotor[1], pMotor[2], pMotor[3]};
	arm_matrix_instance_f32 matMot = { 4, 1, motor };

	for(uint8_t i = 0; i < 4; i++)
		motor[i] *= (robotData.specs.physical.wheelRadius_m * 2.0 * M_PI / 60.0);//*CTRL_MOTOR_TO_WHEEL_RATIO;	// value is now speed over ground [m/s]

	// convert to local velocity
	arm_matrix_instance_f32 matLocal = {3, 1, pLocal};

	float validation[3][4];

	for(int k = 0; k < 3; k++)
	{
		for(int a = 0; a < 4; a++)
		{
			validation[k][a] = MAT_ELEMENT(robotData.math.matMotor2XYW, k, a);
		}
	}

	arm_mat_mult_f32(&robotData.math.matMotor2XYW, &matMot, &matLocal);
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

static float PYTHAGF(float a, float b)
{
	float at = fabsf(a), bt = fabsf(b), ct, result;

    if (at > bt)       { ct = bt / at; result = at * sqrtf(1.0f + ct * ct); }
    else if (bt > 0.0f) { ct = at / bt; result = bt * sqrtf(1.0f + ct * ct); }
    else result = 0.0f;
    return(result);
}

//=============================================================================
//
/**
 * @brief Gerencia as interrupções externas
 *
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Pacote recebido
    if (GPIO_Pin == RADIO_IRQ_Pin &&
        HAL_GPIO_ReadPin(RADIO_IRQ_GPIO_Port, RADIO_IRQ_Pin) == GPIO_PIN_RESET)
    {
        HAL_GPIO_TogglePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin);
        radioData.dataReceived = true;
    }
}

/**
 * @brief Gerencia as interrupções do conversor AD
 *
 * @param hadc
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static const int16_t R9 = 10000, R12 = 1000;
    // Leitura em mV
    robotData.mVbattery =
        uhADCxConvertedValue[0] * 5000 / 4096 * (R9 + R12) / R12;
    // Leitura esperada pelos códigos antigos
    robotData.battery = uhADCxConvertedValue[0] * R12 / (R9 + R12);

    capacitorVoltages[capacitorIndex++] =
        uhADCxConvertedValue[1] * 5000 / 4096 * 151200 / 1200 / 1000;
    if (capacitorIndex >= CAPACITOR_SAMPLES)
    {
        capacitorIndex = 0;
    }

    // static const int16_t R9 = 10000, R12 = 1000;
    // HAL_ADC_Stop_IT(&hadc1);
    // // Leitura em mV
    // robotData.mVbattery =
    //     HAL_ADC_GetValue(&hadc1) * 5000 / 4096 * (R9 + R12) / R12;
    //
    // if (robotData.mVbattery < 11000)
    // {
    //     playSong();
    //     if (robotData.cycles % 400 == 0)
    //     {
    //         resetSong();
    //     }
    // }
    // else
    // {
    //     turnOff();
    // }
    // // Leitura esperada pelos códigos antigos
    // // 11 = (R9 + R12) / R12
    // robotData.battery = HAL_ADC_GetValue(&hadc1) * R12 / (R9 + R12);
    // // robotData.battery = HAL_ADC_GetValue(&hadc1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == microSecTimer)
    {
        delayCount += 50;
        tick(50);
    }
}
