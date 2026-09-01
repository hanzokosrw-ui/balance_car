  
#include "beep.h"   

// Function: Buzzer initialization - 蜂鸣器初始化	 	
void BEEP_GPIO_Config(void)
{		

		// 定义一个GPIO_InitTypeDef类型的结构体
	GPIO_InitTypeDef GPIO_InitStructure;

		// 关闭JTAG接口
	JTAG_Set(JTAG_SWD_DISABLE);    

		// 打开SWD接口 可以利用主板的SWD接口调试
	JTAG_Set(SWD_ENABLE);           

		// 开启控制蜂鸣器的GPIO的端口时钟
	RCC_APB2PeriphClockCmd(BEEP_GPIO_CLK, ENABLE); 

		// 选择要控制蜂鸣器的GPIO															   
	GPIO_InitStructure.GPIO_Pin = BEEP_GPIO_PIN;	

		// 设置GPIO模式为通用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		// 设置GPIO速率为50MHz   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

		// 调用库函数，初始化控制蜂鸣器的GPIO
	GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStructure);			 

		// 关闭蜂鸣器
	GPIO_ResetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN);	 
}



// Function: Buzzer_Alarm - 蜂鸣器报警	 	
//在中断函数调用
void Buzzer_Alarm(u16 count)
{
	static int count_time;
	if(0 == count)
	{
		BEEP_OFF;
	}
	else if(++count_time >= count)
	{
		BEEP_TOGGLE;
		count_time = 0;	
	}
}




// END OF FILE
