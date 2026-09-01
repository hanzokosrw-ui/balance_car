
#include "motor.h"
// Function: PWM_OutPut_TIM_GPIO_Config - 配置PWM输出端口，用于控制电机	 	
static void PWM_OutPut_TIM_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 输出比较通道1 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 输出比较通道2 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 输出比较通道3 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 输出比较通道4 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


// 注意：TIM_TimeBaseInitTypeDef有5个成员，TIM6/TIM7只需初始化TIM_Prescaler和TIM_Period两个

// ----------------   PWM信号 周期和占空比的计算---------------
// ARR ：自动重装载寄存器的值
// CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
// PWM 信号的周期 T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// 占空比P=CCR/(ARR+1)

// Function: PWM_OutPut_TIM_Mode_Config - 配置PWM输出模式，用于控制电机	 	
static void PWM_OutPut_TIM_Mode_Config(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

  // 开启定时器时钟,即内部时钟CK_INT=72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

		// --------------------时基结构体初始化-------------------------

	TIM_TimeBaseInitStruct.TIM_Period = arr;              			//设定计数器自动重装值 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          			//设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        //设置时钟分割
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);      	//初始化定时器

	
		// --------------------输出比较结构体初始化-------------------	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//设置输出极性
	TIM_OC1Init(TIM3,&TIM_OCInitStruct);                	//初始化输出比较参数

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//设置输出极性
	TIM_OC2Init(TIM3,&TIM_OCInitStruct);                 	//初始化输出比较参数

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//设置输出极性
	TIM_OC3Init(TIM3,&TIM_OCInitStruct);                  //初始化输出比较参数

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             		//选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            		//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     		//设置输出极性
	TIM_OC4Init(TIM3,&TIM_OCInitStruct);                  //初始化输出比较参数

	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);   	//CH1使能预装载寄存器
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);   	//CH2使能预装载寄存器
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);   	//CH3使能预装载寄存器
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);   	//CH4使能预装载寄存器

	TIM_ARRPreloadConfig(TIM3, ENABLE);                	//使定时器3在ARR上的预装载寄存器

	TIM_Cmd(TIM3,ENABLE);                              	//使能定时器3
}

void MiniBalance_PWM_Init(u16 arr,u16 psc)
{
	PWM_OutPut_TIM_GPIO_Config();			//GPIO配置
	PWM_OutPut_TIM_Mode_Config(arr,psc);	//模式配置
}
