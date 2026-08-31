#ifndef __PID_H
#define __PID_H

/* ---- balance PID parameters ---- */
#define BALANCE_PID_KP              0.03f
#define BALANCE_PID_KI              0.0f
#define BALANCE_PID_KD              0.02f
#define BALANCE_PID_INTEGRAL_LIMIT  500.0f
#define BALANCE_PID_OUTPUT_LIMIT    98

/* ---- tuning step ---- */
#define BALANCE_PID_KP_STEP             0.01f   /* ?? ? Kp += step */
#define BALANCE_PERIOD_ADJUST_STEP      0.1f    /* ?? ? PERIOD_ADJUST += step */
#define BALANCE_PERIOD_ADJUST_ERROR_STEP 50     /* ?? ? PERIOD_ADJUST_ERROR += step */
#define ERROR_ADJUST  80//xuyaochuyi 100
#define PERIOD_ADJUST  1.4
#define PERIOD_ADJUST_ERROR  600
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float integral_limit;
    float output_limit;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
void pid_reset(pid_t *pid);
int pid_update(pid_t *pid, int target, int measure);

#endif
