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

static FusionEKFConfig configFusionKF = {
	.posNoiseXY = 0.001f,
	.posNoiseW = 0.001f,
	.velNoiseXY = 0.005f,
	.visNoiseXY = 0.05f,
	.visNoiseW = 0.1f,
	.outlierMaxVelXY = 3.0f,
	.outlierMaxVelW = 3.0f,
	.trackingCoeff = 1.0f,
	.visCaptureDelay = 20,
	.fusionHorizon = 35,
	.visionTimeoutMs = 1000,
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
    convertDebugSpeed(&debugData.wheelSpeed3High, &debugData.wheelSpeed3Low,
                      robotData.wheelSpeed[MOTOR_3]);
    convertDebugSpeed(&debugData.wheelSpeed4High, &debugData.wheelSpeed4Low,
                      robotData.wheelSpeed[MOTOR_4]);

    convertDebugSpeed(&debugData.count1High, &debugData.count1Low,
                      robotData.encoderCount[MOTOR_1]);
    convertDebugSpeed(&debugData.count2High, &debugData.count2Low,
                      robotData.encoderCount[MOTOR_2]);
    convertDebugSpeed(&debugData.count3High, &debugData.count3Low,
                      robotData.encoderCount[MOTOR_3]);
    convertDebugSpeed(&debugData.count4High, &debugData.count4Low,
                      robotData.encoderCount[MOTOR_4]);
    convertDebugSpeed(&debugData.posXHigh, &debugData.posXLow,
    				  robotData.state.pos[0]);
    convertDebugSpeed(&debugData.posYHigh, &debugData.posYLow,
					  robotData.state.pos[1]);
    convertDebugSpeed(&debugData.posThetaHigh, &debugData.posThetaLow,
					  robotData.state.pos[2]);
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
	robotData.sensors.acc.linAcc[0] = accel_xyz[0] * 0.061f / 0.10197162129779f / 1000.f + 0.0498709083 - 0.0872829258;
	robotData.sensors.acc.linAcc[1] = accel_xyz[1] * 0.061f / 0.10197162129779f / 1000.f + 0.266912103;
	robotData.sensors.acc.linAcc[2] = accel_xyz[2] * 0.061f / 0.10197162129779f / 1000.f + 0.452083707;

	BSP_GYRO_GetXYZ(gyro_xyz);
	robotData.sensors.gyr.rotVel[0] = (gyro_xyz[0] / 1000) - 0.256533086;// - 0.225843;
	robotData.sensors.gyr.rotVel[1] = (gyro_xyz[1] / 1000) + 0.116062336;// + 0.089630;
	robotData.sensors.gyr.rotVel[2] = (gyro_xyz[2] / 1000) + 0.503718257;// + 0.631300;

	robotData.sensors.vision.pos[0] = robotData.pose.x;
	robotData.sensors.vision.pos[1] = robotData.pose.y;
	robotData.sensors.vision.pos[2] = robotData.pose.theta;

	FusionEKFUpdate(&robotData.sensors, &robotData.state);
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
