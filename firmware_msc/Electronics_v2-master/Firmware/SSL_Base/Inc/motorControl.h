/*
 * motorControl.h
 *
 *  Created on: Sep 20, 2022
 *      Author: leonardo
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#include "main.h"
#include <math.h>

extern TIM_HandleTypeDef* pwmMotor1;
extern TIM_HandleTypeDef* pwmMotor2;
extern TIM_HandleTypeDef* pwmMotor3;
extern TIM_HandleTypeDef* pwmMotor4;
extern TIM_HandleTypeDef* pwmMotorRoller;

typedef enum
{
    MOTOR_UNDEFINED = -1,
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3,
    MOTOR_ROLLER = 4,
    MOTOR_SZ
} MOTOR_ID;

#define OLD_ROBOT

#ifdef OLD_ROBOT
#define WHEEL_DISTANCE 80.5e-3 // Distancia da roda até o centro de massa do robô [m]
// 33 graus
#define SIN33 0.5446390350150270f
#define COS33 0.8386705679454240f
#else
 #define WHEEL_DISTANCE 83e-3 // Distancia da roda até o centro de massa do robô [m]
//#define WHEEL_DISTANCE 80.5e-3 // Distancia da roda até o centro de massa do robô [m]
// 30 graus
#define SIN_ALPHA 0.5f
#define COS_ALPHA 0.8660254037844f

// 45 graus
#define SIN_BETA 0.7071067811865f
#define COS_BETA 0.7071067811865f

//// 33 graus
//#define SIN_ALPHA 0.5446390350150270f
//#define COS_ALPHA 0.8386705679454240f
//
//// 34 graus
//#define SIN_BETA 0.559192903470747f
//#define COS_BETA 0.829037572555042f

// // 22 graus
// #define SIN_ALPHA 374.606593416e-3
// #define COS_ALPHA 927.183854567e-3
//
// // 44 graus
// #define SIN_BETA 694.658370459e-3
// #define COS_BETA 719.339800339e-3
#endif


#define SPEED_LIMIT 260.f  // Limite de velocidade em [rad/s]
#define WHEEL_RADIUS 27e-3 // Raio da roda em [m]
#define MAX_RPM 2500       // Rotação máxima da roda [rpm]
#define MAX_VELOCITY 1.0f  // Velocidade máxima em [m/s]
#define MAX_ROTATION 3.1415f  // Rotação máximo em [rad/s]

#define TRANSMISSION 3 // Redução da transmissão (3:1)
#define RADS_TO_RPM 60.f / (2.f * 3.14159265f)
#define RPM_TO_PWM 100.f / MAX_RPM

#define MOTOR_CYCLES 200 // Tempo que o firmware mantém o comando atual de
                         // velocidade antes de desligar o motor. 200 [ms].
#define MOTOR_INIT                                                             \
    {                                                                          \
        .id = MOTOR_UNDEFINED, .init = false, .cyclesRemaining = 0,            \
        .deadZone = 0, .acceleration = 0, .setPoint = 0, .currentSpeed = 0,    \
        .pwmPin = 0, .pwmPort = NULL, .pwmTimer = NULL, .pwmChannel = 0        \
    }
typedef struct
{
    MOTOR_ID id;
    bool init;
    uint8_t cyclesRemaining;

    uint8_t deadZone;
    uint8_t acceleration;

    int8_t setPoint;
    int8_t currentSpeed;

    uint16_t pwmPin;
    GPIO_TypeDef* pwmPort;

    TIM_HandleTypeDef* pwmTimer;
    uint32_t pwmChannel;

} Motor_t;

static Motor_t motors[MOTOR_SZ] = {MOTOR_INIT, MOTOR_INIT, MOTOR_INIT,
                                   MOTOR_INIT, MOTOR_INIT};

void initMotors();

void updateFeedback(const int16_t* _m1, const int16_t* _m2, const int16_t* _m3,
                    const int16_t* _m4);

void setMotorsCommand(const int8_t* _vx, const int8_t* _vy, const int8_t* _vw,
                      bool _useDeadZone);

void setRollerCommand(const int8_t* _v);

void runMotorControl();

#endif /* INC_MOTORCONTROL_H_ */
