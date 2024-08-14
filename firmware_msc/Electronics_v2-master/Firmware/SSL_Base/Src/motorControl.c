/*
 * motorControl.c
 *
 *  Created on: Sep 20, 2022
 *      Author: leonardo
 */

#include "motorControl.h"
#include "main.h"
#include "pid.h"

#include "stm32f4xx_hal_def.h"
#include <math.h>
#include <stdlib.h>

// #define USE_ENCODER

void initMotors()
{
    TIM_HandleTypeDef* PWM_TIMERS[MOTOR_SZ] = {pwmMotor1, pwmMotor2, pwmMotor3,
                                               pwmMotor4, pwmMotorRoller};

    uint16_t PWM_PINS[MOTOR_SZ] = {PWM_M1_Pin, PWM_M2_Pin, PWM_M3_Pin,
                                   PWM_M4_Pin, PWM_ROLLER_Pin};
    //            PWM_M4_Pin, PWM_Roller_Pin};

    GPIO_TypeDef* PWM_PORTS[MOTOR_SZ] = {PWM_M1_GPIO_Port, PWM_M2_GPIO_Port,
                                         PWM_M3_GPIO_Port, PWM_M4_GPIO_Port,
                                         PWM_ROLLER_GPIO_Port};
    //                                             PWM_Roller_GPIO_Port};

    uint32_t PWM_CHANNEL[MOTOR_SZ] = {TIM_CHANNEL_1, TIM_CHANNEL_2,
                                      TIM_CHANNEL_2, TIM_CHANNEL_4,
                                      TIM_CHANNEL_3};

    for (uint8_t id = MOTOR_1; id < MOTOR_SZ; ++id)
    {
        motors[id].id = id;

        motors[id].deadZone = 10;
        motors[id].acceleration = 1;

        motors[id].setPoint = 0;
        motors[id].currentSpeed = 0;

        motors[id].pwmPin = PWM_PINS[id];
        motors[id].pwmPort = PWM_PORTS[id];

        motors[id].pwmTimer = PWM_TIMERS[id];
        motors[id].pwmChannel = PWM_CHANNEL[id];

        if (HAL_TIM_PWM_Start(motors[id].pwmTimer, motors[id].pwmChannel) !=
            HAL_OK)
        {
            Error_Handler();
        }
        motors[id].init = true;
    }

    // Garante que o robô não vai sair andando ao iniciar
    //    setMotorsCommand(0, 0, 0);
    runMotorControl();
}

void updateFeedback(const int16_t* _m1, const int16_t* _m2, const int16_t* _m3,
                    const int16_t* _m4)
{
    pidControl[MOTOR_1].wheelSpeed = *_m1;
    pidControl[MOTOR_2].wheelSpeed = *_m2;
    pidControl[MOTOR_3].wheelSpeed = *_m3;
    pidControl[MOTOR_4].wheelSpeed = *_m4;
}

void setMotorsCommand(const int8_t* _vx, const int8_t* _vy, const int8_t* _vw,
                      bool _useDeadZone)
{
    // Calcula as velocidades de cada motor a partir da cinemática direta
    float motorSpeeds[MOTOR_SZ - 1];
    float vx = *_vx * MAX_VELOCITY / 100, vy = *_vy * MAX_VELOCITY / 100,
          vw = *_vw * MAX_ROTATION / 100;

    // Limita o Vx em [-1; 1] m/s
    // vx = min(1.5f, vx);
    // vx = max(-1.5f, vx);

#ifdef OLD_ROBOT
    motorSpeeds[MOTOR_1] =
        TRANSMISSION * (-SIN33 * vx - COS33 * vy + vw * WHEEL_DISTANCE) / WHEEL_RADIUS;
    motorSpeeds[MOTOR_2] =
        TRANSMISSION * (SIN33 * vx - COS33 * vy + vw * WHEEL_DISTANCE) / WHEEL_RADIUS;
    motorSpeeds[MOTOR_3] =
        TRANSMISSION * (SIN33 * vx + COS33 * vy + vw * WHEEL_DISTANCE) / WHEEL_RADIUS;
    motorSpeeds[MOTOR_4] =
        TRANSMISSION * (-SIN33 * vx + COS33 * vy + vw * WHEEL_DISTANCE) / WHEEL_RADIUS;
#else
    motorSpeeds[MOTOR_1] =
        TRANSMISSION * (-SIN_ALPHA * vx - COS_ALPHA * vy + vw * WHEEL_DISTANCE) / WHEEL_RADIUS;
    motorSpeeds[MOTOR_2] =
        TRANSMISSION * (SIN_BETA * vx - COS_BETA * vy + vw * WHEEL_DISTANCE) / WHEEL_RADIUS;
    motorSpeeds[MOTOR_3] =
        TRANSMISSION * (SIN_BETA * vx + COS_BETA * vy + vw * WHEEL_DISTANCE) / WHEEL_RADIUS;
    motorSpeeds[MOTOR_4] =
        TRANSMISSION * (-SIN_ALPHA * vx + COS_ALPHA * vy + vw * WHEEL_DISTANCE) / WHEEL_RADIUS;
#endif /* ifdef OLD_ROBOT */
    // Limitacao da velocidade das rodas, caso alguma seja maior que 100
    float maxSpeed =
        max(max(fabsf(motorSpeeds[MOTOR_1]), fabsf(motorSpeeds[MOTOR_2])),
            max(fabsf(motorSpeeds[MOTOR_3]), fabsf(motorSpeeds[MOTOR_4])));
    float minSpeed =
        min(min(fabsf(motorSpeeds[MOTOR_1]), fabsf(motorSpeeds[MOTOR_2])),
            min(fabsf(motorSpeeds[MOTOR_3]), fabsf(motorSpeeds[MOTOR_4])));
    float adjustFactor = 1;
    if (maxSpeed > SPEED_LIMIT)
    {
        adjustFactor = maxSpeed / SPEED_LIMIT;
    }

    // Fator para calibração de motores diferentes no mesmo robô
    const float factor[MOTOR_SZ] = {1.0, 1.0, 1.0, 1.0};

    for (int8_t id = MOTOR_1; id < MOTOR_SZ - 1; ++id)
    {
        if (minSpeed < motors[id].deadZone && _useDeadZone)
        {
            motorSpeeds[id] += motors[id].deadZone * motorSpeeds[id] /
                               fabsf(motorSpeeds[id] + 1e-10f);
        }

#ifdef USE_ENCODER
        motorSpeeds[id] = factor[id] * RADS_TO_RPM * motorSpeeds[id] / adjustFactor;
        setTargetSpeed(&pidControl[id], motorSpeeds[id]);
#else
        motors[id].setPoint =
            factor[id] * RADS_TO_RPM * RPM_TO_PWM * motorSpeeds[id] / adjustFactor;
#endif

        motors[id].cyclesRemaining = MOTOR_CYCLES;
    }
}

void setRollerCommand(const int8_t* _v)
{
    int8_t cmd = *_v;

    if (cmd < 0)
    {
        cmd *= -1;
    }

    if (cmd > 100)
    {
        cmd = 100;
    }

    motors[MOTOR_ROLLER].setPoint = -cmd;
    motors[MOTOR_ROLLER].cyclesRemaining = MOTOR_CYCLES;
}

void runMotorControl()
{
    for (int8_t id = MOTOR_1; id < MOTOR_SZ; ++id)
    {
        if (motors[id].init == false)
        {
            return;
        }
        if (motors[id].cyclesRemaining > 0)
        {
            motors[id].cyclesRemaining--;
        }
    }

    for (int8_t id = MOTOR_1; id < MOTOR_SZ; ++id)
    {
#ifdef USE_ENCODER
        // Nao tem PID no roller
        if (id <= MOTOR_4)
        {
            motors[id].setPoint = run(&pidControl[id]) * RPM_TO_PWM;
        }
#endif
        int pwm = motors[id].setPoint;

        if (motors[id].cyclesRemaining == 0)
        {
            pwm = 0;
            HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            // Para evitar chegar no 0% ou 100% de Duty cycle
            if (pwm >= 100)
            {
                pwm = 99;
            }

            else if (pwm <= -100)
            {
                pwm = -99;
            }
        }
        // PWM        :   0----------100----------200
        // Velocidade : -100----------0-----------100
        pwm += 100;

        uint32_t adjustSetpoint =
            motors[id].pwmTimer->Instance->ARR * pwm / 200;

        // Limita ao valor máximo do registrador
        if (adjustSetpoint > motors[id].pwmTimer->Instance->ARR)
        {
            adjustSetpoint = motors[id].pwmTimer->Instance->ARR;
        }
        __HAL_TIM_SET_COMPARE(motors[id].pwmTimer, motors[id].pwmChannel,
                              adjustSetpoint);
    }
}
