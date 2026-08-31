#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include <stdio.h>
#include "OLED.h"
#include "MPU6050.h"
#include "kalman.h"
#include "pid.h"
#include "motor.h"
#include "key.h"

#define CONTROL_PERIOD_MS  10    /* 10ms = 100Hz balance loop */

static void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio_initstruct.Pin   = GPIO_PIN_5;
    gpio_initstruct.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_initstruct.Pull  = GPIO_PULLUP;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);

    gpio_initstruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOE, &gpio_initstruct);
}

int main(void)
{
    uint32_t last_tick;
    mpu6050_data_t mpu_data;
    kalman_t       kalman_pitch;
    pid_t          balance_pid;
    uint8_t        filters_inited = 0;
    float          kalman_angle = 0.0f;
    float          period_adjust = PERIOD_ADJUST;
    int            period_adjust_error = PERIOD_ADJUST_ERROR;

    /* ---- system init ---- */
    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);
    led_init();
    OLED_Init();
    KEY_Init();

    /* ---- MPU6050 init ---- */
    mpu6050_init();
    printf("MPU6050 initialized.\r\n");

    /* ---- motor init ---- */
    motor_init();

    /* ---- PID init (target=0 upright, measure=kalman_angle*100) ---- */
    pid_init(&balance_pid,
             BALANCE_PID_KP, BALANCE_PID_KI, BALANCE_PID_KD,
             BALANCE_PID_INTEGRAL_LIMIT, BALANCE_PID_OUTPUT_LIMIT);

    last_tick = HAL_GetTick();

    while (1)
    {
				int pid_out;
        uint32_t now = HAL_GetTick();

        if (now - last_tick >= CONTROL_PERIOD_MS)
        {
            float dt = (float)(now - last_tick) / 1000.0f;
            last_tick = now;

            if (mpu6050_read_data(&mpu_data))
            {
                /* DMP pitch (fAY) + gyro X rate */
                float dmp_pitch = fAY;
                float gyro_rate = (float)gx / 16.4f;

                if (!filters_inited)
                {
                    kalman_init(&kalman_pitch, dmp_pitch);
                    filters_inited = 1;
                }

                kalman_angle = kalman_update(&kalman_pitch, dmp_pitch, gyro_rate, dt);

                /* PID: target=0, measure scaled x100 for integer resolution */
                pid_out = pid_update(&balance_pid, 0, (int)(kalman_angle * 100.0f));

                /* output limit (belt-and-suspenders; motor_set_* also clamps) */
                

                /* negate output: forward tilt ? forward motor (negative feedback)
                   right motor is mechanically inverted ? drive with opposite sign */
                motor_set_left(-pid_out);
                motor_set_right(pid_out);
                //   motor_set_left(-57);
                //  motor_set_right(57);???
            }

            /* ---- key scan & parameter tuning ---- */
            {
                int key_event = KEY_Scan();
                if (key_event == KEY_EVENT_LONG_PRESS) {
                    balance_pid.kp += BALANCE_PID_KP_STEP;
                } else if (key_event == KEY_EVENT_CLICK) {
                    period_adjust += BALANCE_PERIOD_ADJUST_STEP;
                } else if (key_event == KEY_EVENT_DOUBLE_CLICK) {
                    period_adjust_error += BALANCE_PERIOD_ADJUST_ERROR_STEP;
                }
            }

            /* ---- OLED display ---- */
            OLED_Clear();
            OLED_Printf(0, 0,  OLED_6X8, "Pitch:%5.1f", kalman_angle);
            OLED_Printf(0, 8,  OLED_6X8, "Kp:%5.2f",  balance_pid.kp);
            OLED_Printf(0, 16, OLED_6X8, "PA:%5.1f",  period_adjust);
            OLED_Printf(0, 24, OLED_6X8, "PA_E:%d",   period_adjust_error);
            OLED_Printf(0, 32, OLED_6X8, "Out:%d",    pid_out);
            OLED_Update();
        }
    }
}

