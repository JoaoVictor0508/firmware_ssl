/**
 * @file encoder.c
 * @brief Implementação da leitura dos encoders
 */

#include "encoder.h"
#include "main.h"
#include "stm32f4xx_hal_tim.h"

void initEncoders()
{
    uint16_t ENCODER_CHA[MOTOR_SZ - 1] = {ENC_M1_A_Pin, ENC_M2_A_Pin,
                                          ENC_M3_A_Pin, ENC_M4_A_Pin};
    uint16_t ENCODER_CHB[MOTOR_SZ - 1] = {ENC_M1_B_Pin, ENC_M2_B_Pin,
                                          ENC_M3_B_Pin, ENC_M4_B_Pin};
    TIM_HandleTypeDef* ENCODER_TIMERS[MOTOR_SZ - 1] = {encoderM1, encoderM2,
                                                       encoderM3, encoderM4};

    for (uint8_t id = MOTOR_1; id < MOTOR_SZ - 1; ++id)
    {
        encoders[id].id = id;

        encoders[id].encoderCHA = ENCODER_CHA[id];
        encoders[id].encoderCHB = ENCODER_CHB[id];
        encoders[id].timer = ENCODER_TIMERS[id];

        encoders[id].count = 0;
        encoders[id].totalCount = 0;
        encoders[id].speed = 0;
        encoders[id].t = 0;

        if (encoders[id].timer != NULL)
        {
            if (HAL_TIM_Encoder_Start(encoders[id].timer, TIM_CHANNEL_ALL) !=
                HAL_OK)
            {
                Error_Handler();
            }
            encoders[id].init = true;
        }
    }
}

void updateEncoders()
{
    float dt;
    float rotations;
    float speed;

    for (uint8_t id = MOTOR_1; id < MOTOR_SZ - 1; ++id)
    {
        if (encoders[id].init)
        {
            encoders[id].count = __HAL_TIM_GET_COUNTER(encoders[id].timer);
            encoders[id].totalCount += encoders[id].count / 2;
            __HAL_TIM_SET_COUNTER(encoders[id].timer, 0);

            dt = (__HAL_TIM_GET_COUNTER(microSecTimer) - encoders[id].t) *
                 1e-6; // us
            encoders[id].t = __HAL_TIM_GET_COUNTER(microSecTimer);

            // Divide por 2 porque o timer conta dobrado (canal A + B)
            rotations = encoders[id].count /
                        (2.f * WHEEL_TRANSMISSION * ENCODER_PPR); // Rotações
            speed = -(rotations / dt) * 60;

//            encoders[id].speed = speed;

            // if(abs(speed - encoders[id].speed) < 50)
            //
            // {
            encoders[id].speed = filter(&encoders[id], speed);
            // }
        }
    }
}

void getSpeed(int16_t* _M1, int16_t* _M2, int16_t* _M3, int16_t* _M4)
{
    *_M1 = encoders[MOTOR_1].speed;
    *_M2 = encoders[MOTOR_2].speed;
    *_M3 = encoders[MOTOR_3].speed;
    *_M4 = encoders[MOTOR_4].speed;
}

void getCounts(int16_t* _C1, int16_t* _C2, int16_t* _C3, int16_t* _C4)
{
    *_C1 = encoders[MOTOR_1].totalCount;
    *_C2 = encoders[MOTOR_2].totalCount;
    *_C3 = encoders[MOTOR_3].totalCount;
    *_C4 = encoders[MOTOR_4].totalCount;
}

float filter(Encoder_t* _encoder, float _speed)
{
    // Não acredita em 100% da leitura da nova velocidade dependendo do
    // quanto
    // ela mudou do instante de tempo anterior para agora.
    //
    // 0.00001 é para evitar divisão por 0
    float belief =
        1.f -
        min(max(abs((_speed - _encoder->speed) / (_encoder->speed + 0.00001)),
                0.3f),
            0.9);

    float result = (1.f - belief) * _encoder->speed + belief * _speed;
    _encoder->speedHistory[_encoder->historyIndex++] = result;

    if (_encoder->historyIndex >= SAMPLE_SIZE)
    {
        _encoder->historyIndex = 0;
    }

    result =
        convolve(_encoder->speedHistory, MA_FILTER, _encoder->speedFiltered);
    return result;
}

float convolve(float h[SAMPLE_SIZE], float x[MOVING_AVERAGE],
               float y[CONV_RESULT])
{
    int nconv = CONV_RESULT;
    int i, j, h_start, x_start, x_end;

    for (uint8_t n = 0; n < CONV_RESULT; n++)
    {
        y[n] = 0;
    }

    for (i = 0; i < nconv; i++)
    {
        x_start = max(0, i - SAMPLE_SIZE + 1);
        x_end = min(i + 1, MOVING_AVERAGE);
        h_start = min(i, SAMPLE_SIZE - 1);
        for (j = x_start; j < x_end; j++)
        {
            y[i] += h[h_start--] * x[j];
        }
    }
    return y[(CONV_RESULT) / 2];
}
