#include "pid.h"

void setGains(PID_t* _obj, float _kp, float _ki, float _kd)
{
    _obj->kP = _kp;
    _obj->kI = _ki;
    _obj->kD = _kd;
}

void setTargetSpeed(PID_t* _obj, int32_t _target)
{
    if (abs(_obj->setpoint - _target) > 50)
    {
        _obj->errorSum = 0;
    }
    _obj->setpoint = _target;
}

float run(PID_t* _obj)
{
    float output;
    const double error = _obj->setpoint - _obj->wheelSpeed;
    const double errorDerivative =
        (error - _obj->previousError) / _obj->sampleTime;

    _obj->errorSum += error;
    output = error * _obj->kP + _obj->errorSum * _obj->kI -
             errorDerivative * _obj->kD;

    // Nao somar no integrador quando a saída estiver saturada para prevenir
    // wind-up
    if (output >= _obj->maxVelocity || output <= -_obj->maxVelocity)
    {
        _obj->errorSum -= error;
    }

    _obj->previousError = error;
    return output;
}
