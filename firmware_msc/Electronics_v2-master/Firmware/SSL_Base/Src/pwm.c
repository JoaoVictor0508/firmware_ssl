/**
 * @file pwm.c
 * @brief PWM via software
 */

#include "pwm.h"
#include "stm32f4xx_hal_gpio.h"

void initPWM()
{
    pwmBuzzer.init = true;
    pwmBuzzer.pin = PWM_BUZZER_Pin;
    pwmBuzzer.port = PWM_BUZZER_GPIO_Port;
//    pwmBuzzer.pin = PWM_Buzzer_Pin;
//    pwmBuzzer.port = PWM_Buzzer_GPIO_Port;
}

void tick(uint16_t _pulses)
{
    if (pwmBuzzer.init && pwmBuzzer.play)
    {
        pwmBuzzer.counter += _pulses;
        pwmBuzzer.noteTime += _pulses;

        if (pwmBuzzer.counter * 2 == pwmBuzzer.counterValue)
        {
            HAL_GPIO_TogglePin(pwmBuzzer.port, pwmBuzzer.pin);
        }
        else if (pwmBuzzer.counter >= pwmBuzzer.counterValue)
        {
            HAL_GPIO_TogglePin(pwmBuzzer.port, pwmBuzzer.pin);
            pwmBuzzer.counter = 0;
        }

        if (pwmBuzzer.noteTime >= pwmBuzzer.noteDuration)
        {
            HAL_GPIO_WritePin(pwmBuzzer.port, pwmBuzzer.pin, GPIO_PIN_RESET);
        }
        if (pwmBuzzer.noteTime >= pwmBuzzer.noteDuration + pwmBuzzer.noteDelay)
        {
            pwmBuzzer.noteEnded = true;
        }
    }
}

void playNote(int16_t _note, uint32_t _duration, uint32_t _delay)
{
    if (pwmBuzzer.init && pwmBuzzer.noteEnded == true)
    {
        pwmBuzzer.play = true;
        pwmBuzzer.frequency = _note;
        pwmBuzzer.noteDuration = _duration;
        pwmBuzzer.noteDelay = _delay;

        pwmBuzzer.counterValue = (1.0e6 / pwmBuzzer.frequency);
        pwmBuzzer.counter = 0;
        pwmBuzzer.noteTime = 0;
        pwmBuzzer.noteEnded = false;
    }
}

void playSong()
{
    uint16_t* song = &LOW_BATT_SONG[0];
    uint16_t* tempo = &LOW_BATT_TEMPO[0];
    uint16_t songSize = sizeof(LOW_BATT_SONG) / sizeof(uint16_t);

    if (pwmBuzzer.songSize == 0)
    {
        pwmBuzzer.songSize = songSize;
    }

    if (pwmBuzzer.songIndex < pwmBuzzer.songSize)
    {
        uint32_t noteDuration = 1000 / tempo[pwmBuzzer.songIndex];
        if (noteDuration != pwmBuzzer.noteDuration &&
            song[pwmBuzzer.songIndex] != pwmBuzzer.frequency)
        {
            playNote(song[pwmBuzzer.songIndex], noteDuration * 1000,
                     noteDuration * 1300);
        }

        if (pwmBuzzer.noteEnded)
        {
            pwmBuzzer.songIndex++;
        }
    }
    //    else
    //    {
    //        turnOff();
    //    }
}

void resetSong()
{
    pwmBuzzer.songIndex = 0;
}

void turnOff()
{
    if (pwmBuzzer.init)
    {
        pwmBuzzer.play = false;
    }
}
