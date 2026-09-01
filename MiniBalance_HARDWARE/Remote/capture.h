
#ifndef __CAPTURE_H
#define	__CAPTURE_H



// -------------超声波测距程序使用-----------


#include "stm32f10x.h"
#include "control.h"





//超声波和航模遥控的宏定义，只能使用其一
//选择不需要使用的注释掉
#define Distance_Capture


//超声波触发引脚
//使用端口PC15触发（普通IO）,超声波模块捕获是定时器2通道2//超声波模块1
#define 			CAPTURE_TRIG_GPIO_CLK1			RCC_APB2Periph_GPIOC
#define            	CAPTURE_TRIG_PORT1         		GPIOC
#define            	CAPTURE_TRIG_PIN1           	GPIO_Pin_15

#define 			TRIG_HIGH1						PCout(15) = 1
#define 			TRIG_LOW1						PCout(15) = 0

//使用端口PA12触发（普通IO）,超声波模块捕获是定时器2通道3//超声波模块2
#define 			CAPTURE_TRIG_GPIO_CLK2			RCC_APB2Periph_GPIOA
#define            	CAPTURE_TRIG_PORT2          	GPIOA
#define            	CAPTURE_TRIG_PIN2          		GPIO_Pin_12

#define 			TRIG_HIGH2						PAout(12) = 1
#define 			TRIG_LOW2						PAout(12) = 0

//使用端口PB13触发（普通IO）,超声波模块捕获是定时器2通道4//超声波模块3
#define 			CAPTURE_TRIG_GPIO_CLK3			RCC_APB2Periph_GPIOB
#define            	CAPTURE_TRIG_PORT3          	GPIOB
#define            	CAPTURE_TRIG_PIN3          		GPIO_Pin_13

#define 			TRIG_HIGH3						PBout(13) = 1
#define 			TRIG_LOW3						PBout(13) = 0

//使用端口PB12触发（普通IO）,超声波模块捕获是定时器1通道4//超声波模块4
#define 			CAPTURE_TRIG_GPIO_CLK4			RCC_APB2Periph_GPIOB
#define            	CAPTURE_TRIG_PORT4         		GPIOB
#define            	CAPTURE_TRIG_PIN4          		GPIO_Pin_12

#define 			TRIG_HIGH4						PBout(12) = 1
#define 			TRIG_LOW4						PBout(12) = 0

//使用高电平捕获功能
//定时器TIM2—CH2,CH3,Ch4
#define 			CAPTURE_TIM2_CH2_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH2_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH2_PIN           	GPIO_Pin_1

#define 			CAPTURE_TIM2_CH3_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH3_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH3_PIN           	GPIO_Pin_2

#define 			CAPTURE_TIM2_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM2_CH4_PORT          	GPIOA
#define            	CAPTURE_TIM2_CH4_PIN           	GPIO_Pin_3


#define 			CAPTURE_TIM2					TIM2
#define            	CAPTURE_TIM2_APBxClock_FUN     	RCC_APB1PeriphClockCmd
#define            	CAPTURE_TIM2_CLK               	RCC_APB1Periph_TIM2
#define            	CAPTURE_TIM2_IRQ               	TIM2_IRQn
#define           	CAPTURE_TIM2_IRQHandler       	TIM2_IRQHandler
//#define 			CAPTURE_TIM2_CHx					TIM_Channel_2
//#define 			CAPTURE_TIM_IT_CCX 				TIM_IT_CC2//捕获通道

//使用TIM1_CH4
#define 			CAPTURE_TIM1_CH4_GPIO_CLK		RCC_APB2Periph_GPIOA
#define            	CAPTURE_TIM1_CH4_PORT          	GPIOA
#define            	CAPTURE_TIM1_CH4_PIN           	GPIO_Pin_11


#define 			CAPTURE_TIM1					TIM1
#define            	CAPTURE_TIM1_APBxClock_FUN     	RCC_APB2PeriphClockCmd
#define            	CAPTURE_TIM1_CLK               	RCC_APB2Periph_TIM1
#define            	CAPTURE_TIM1_CC_IRQn             TIM1_CC_IRQn
#define            	CAPTURE_TIM1_UP_IRQn            TIM1_UP_IRQn
#define           	CAPTURE_TIM1_CC_IRQHandler      TIM1_CC_IRQHandler
#define           	CAPTURE_TIM1_UP_IRQHandler      TIM1_UP_IRQHandler









//// 获取捕获寄存器值函数宏定义
//#define            CAPTURE_TIM_GetCapturex_FUN                 TIM_GetCapture2
//// 捕获信号极性函数宏定义
//#define            CAPTURE_TIM_OCxPolarityConfig_FUN           TIM_OC2PolarityConfig
// 测量的起始边沿
#define            CAPTURE_TIM_STRAT_ICPolarity                TIM_ICPolarity_Rising
// 测量的结束边沿
#define            CAPTURE_TIM_END_ICPolarity                  TIM_ICPolarity_Falling
//光速宏定义
#define 			Light_Speed								   340

// 定时器输入捕获用户自定义变量结构体声明
typedef struct
{   
	uint8_t   Capture_FinishFlag;   // 捕获结束标志位
	uint8_t   Capture_StartFlag;    // 捕获开始标志位
	int  	  Capture_CcrValue;     // 捕获寄存器的值
	uint16_t  Capture_Period;       // 自动重装载寄存器更新标志 
}TIM_ICUserValueTypeDef;

 
void Read_Distane(void);        
void Distance_Cap_Init(u16 arr,u16 psc);


#endif
