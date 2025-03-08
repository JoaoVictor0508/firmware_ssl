/**
 * @file encoder.h
 * @brief Implementação da leitura dos encoders
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
#include "motorControl.h"

extern TIM_HandleTypeDef* encoderM1;
extern TIM_HandleTypeDef* encoderM2;
extern TIM_HandleTypeDef* encoderM3;
extern TIM_HandleTypeDef* encoderM4;

extern TIM_HandleTypeDef* microSecTimer;

#define ENCODER_INIT                                                           \
    {                                                                          \
        .id = MOTOR_UNDEFINED, .init = false, .encoderCHA = 0,                 \
        .encoderCHB = 0, .timer = NULL, .count = 0, .totalCount = 0,        \
        .t = 0, .speed = 0, .historyIndex = 0, .speedHistory = {},             \
        .speedFiltered = {}                                                    \
    }

#define WHEEL_TRANSMISSION 3 // Redução de 3 : 1
#define ENCODER_PPR 1000
#define SAMPLE_SIZE 4
#define MOVING_AVERAGE 4
#define CONV_RESULT (SAMPLE_SIZE + MOVING_AVERAGE - 1)

typedef struct
{
    MOTOR_ID id;
    bool init;

    uint16_t encoderCHA;
    uint16_t encoderCHB;
    TIM_HandleTypeDef* timer;

    int16_t count;
    int16_t totalCount;
    uint32_t t;
    float speed; // Velocidade em [rpm]
    float speedHistory[SAMPLE_SIZE];
    float speedFiltered[CONV_RESULT];
    uint8_t historyIndex;
} Encoder_t;

static float MA_FILTER[MOVING_AVERAGE] = {
    1.f / MOVING_AVERAGE, 1.f / MOVING_AVERAGE, 1.f / MOVING_AVERAGE,
    1.f / MOVING_AVERAGE, 1.f / MOVING_AVERAGE, 1.f / MOVING_AVERAGE,
    1.f / MOVING_AVERAGE, 1.f / MOVING_AVERAGE, 1.f / MOVING_AVERAGE,
    1.f / MOVING_AVERAGE};
static Encoder_t encoders[MOTOR_SZ - 1] = {ENCODER_INIT, ENCODER_INIT,
                                           ENCODER_INIT, ENCODER_INIT};

void initEncoders();

void updateEncoders();

void getSpeed(int16_t* _M1, int16_t* _M2, int16_t* _M3, int16_t* _M4);

void getCounts(int16_t* _C1, int16_t* _C2, int16_t* _C3, int16_t* _C4);

float filter(Encoder_t* _encoder, float _speed);

float convolve(float h[SAMPLE_SIZE], float x[MOVING_AVERAGE],
               float y[CONV_RESULT]);

#endif /* INC_ENCODER_H_ INC_ENCODER_H_ */
