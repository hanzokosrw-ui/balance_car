#include "led.h"
// Function: Led initialization - LED接口初始化
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	            //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOC, &GPIO_InitStructure);	//根据设定参数初始化GPIOA 
}

// Function: Led flashing - LED闪烁
void Led_Flash(u16 time)
{
	 static int temp;
	 if(0==time) LED=1;
	 else 	if(++temp==time)	LED=~LED,temp=0;
}
