/**
 * @file pid.h
 * @author Leonardo da Silva Costa - SocialDroids
 * @version V1.0.0
 * @created 02/05/2023
 * @brief Controlador PID
 */

#ifndef INC_PID_H
#define INC_PID_H

#include "main.h"
#include "motorControl.h"
#include "ssl_main.h"
#include <math.h>

#define PID_INIT                                                               \
    {                                                                          \
        .kP = 1, .kI = 0, .kD = 0, .wheelSpeed = 0, .errorSum = 0,             \
        .setpoint = 0, .previousError = 0, .sampleTime = LOOP_uS,              \
        .maxVelocity = MAX_RPM                                                 \
    }

/**
 * @brief Implementação de um controle PID para as duas rodas do robô.
 *
 */
typedef struct PID_t
{
    float kP, kI, kD;    // PID gains
    float sampleTime;    // Sample update period [ms]
    int32_t maxVelocity; // Used to prevent wind-up [m/s]

    int32_t setpoint;   // [m/s]
    int32_t wheelSpeed; // Wheel speeds [m/s]

    int32_t errorSum;      // Sum of the error for the integral term
    int32_t previousError; // Previous value of the error for the
                           // derivative term
} PID_t;

static PID_t pidControl[MOTOR_SZ - 1] = {PID_INIT, PID_INIT, PID_INIT,
                                         PID_INIT};

/**
 * @brief Configura os ganhos do controlador.
 *
 * @param _kp Ganho proporcional.
 * @param _ki Ganho integral.
 * @param _kd Ganho derivativo.
 */
void setGains(PID_t* _obj, float _kp, float _ki, float _kd);

/**
 * @brief Configura o setpoint de velocidade de cada roda.
 *
 * @param _target Velocidade do motor [rpm].
 */
void setTargetSpeed(PID_t* _obj, int32_t _target);

/**
 * @brief Roda o compilador.
 */
float run(PID_t* _obj);

#endif // INC_PID_H
