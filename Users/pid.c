#include "pid.h"

static float limit_float(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

int pid_update(pid_t *pid, int target, int measure)
{
    float error;
    float derivative;
    float output;

    error = (float)(target - measure)-ERROR_ADJUST;
    pid->integral += error;
    pid->integral = limit_float(pid->integral, pid->integral_limit);

    derivative = error - pid->last_error;
    pid->last_error = error;
    if(error<300 && error>-300)//因为error是角度差的100倍
    {
        output=0;
        return (int)output;
    }//小角度不要偏，因为存在传感器误差

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if(error>PERIOD_ADJUST_ERROR || error<-PERIOD_ADJUST_ERROR)
    {
        output*=PERIOD_ADJUST;
    }//大角度加大输出，防止翻车
    if (output > BALANCE_PID_OUTPUT_LIMIT)  output = BALANCE_PID_OUTPUT_LIMIT;
    if (output < -BALANCE_PID_OUTPUT_LIMIT) output = -BALANCE_PID_OUTPUT_LIMIT;
    /* linear map: [1, LIMIT]→[55, LIMIT], [-LIMIT,-1]→[-LIMIT,-55] */
    if (output > 0)
        output = 55 + output * (BALANCE_PID_OUTPUT_LIMIT - 55) / BALANCE_PID_OUTPUT_LIMIT;
    else if (output < 0)
        output = -55 + output * (BALANCE_PID_OUTPUT_LIMIT - 55) / BALANCE_PID_OUTPUT_LIMIT;
    return (int)output;//此处是实验得来，加负号
}
