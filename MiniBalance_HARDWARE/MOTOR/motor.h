#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

//这里是电机输出的PWM          //PWMX_IN1 为PWM输入时，PWMX_IN2 没有PWM输入时，车轮正转快衰竭
                               //PWMX_IN1 为1输入时，PWMX_IN2 有PWM输入时，车轮正转慢衰竭
#define PWMA_IN1 TIM3->CCR1   
#define PWMA_IN2 TIM3->CCR2   
#define PWMB_IN1 TIM3->CCR3
#define PWMB_IN2 TIM3->CCR4



void MiniBalance_PWM_Init(u16 arr,u16 psc);
#endif
