#include "key.h"
// Function: Key initialization - 按键初始化
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOC, ENABLE); //使能PA端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	            //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;         //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	            //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;         //浮空输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
} 

// Function: Mode Choose - 小车模式选择
void Mode_Choose(void)
{
	switch(User_Key_Scan())
		{
			//单击按键可以切换到
			//0.普通遥控模式
			//7.超声波避障模式
			//8.超声波跟随模式
			//9.红外循迹模式
			case Click:
				if(Mode >= IRDM_Line_Patrol_Mode)
					TeachRemote_SetMode(Normal_Mode); // 保存记录/终止播放，按键不进入10、11
				else if(Mode == Normal_Mode)
					TeachRemote_SetMode(Ultrasonic_Avoid_Mode); //跳过已禁用的ROS模式
				else
					TeachRemote_SetMode(Mode + 1);
				break;
			default:break;
		}
}

// Function: User_Key_Scan - 用户按键检测
//放在5ms中断中调用
uint8_t User_Key_Scan(void)
{
	static u16 count_time = 0;					//计算按下的时间，每5ms加1
	static u8 key_step = 0;						//记录此时的步骤
	switch(key_step)
	{
		case 0:
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == KEY_ON )
				key_step++;						//检测到有按键按下，进入下一步
			break;
		case 1:
			if((++count_time) == 5)				//延时消抖
			{
				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == KEY_ON )//按键确实按下了
					key_step++,count_time = 0;	//进入下一步
				else
					count_time = 0,key_step = 0;//否则复位
			}
			break;
		case 2:
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == KEY_ON )
				count_time++;					//计算按下的时间
			else 								//此时已松开手
				key_step++;						//进入下一步
			break;
		case 3:									//此时看按下的时间，来判断是长按还是短按
			if(count_time > 400)				//在5ms中断中调用，故按下时间若大于400*5 = 2000ms（大概值）
			{							
				key_step = 0;					//标志位复位
				count_time = 0;
				return Long_Press;				//返回 长按 的状态 
 			}
			else if(count_time > 5)				//此时是单击了一次
			{
				key_step++;						//此时进入下一步，判断是否是双击
				count_time = 0;					//按下的时间清零
			}
			else
			{
				key_step = 0;
				count_time = 0;	
			}
			break;
		case 4:									//判断是否是双击或单击
			if(++count_time >20&&count_time<70)				//5*50 = 250ms内判断按键是否按下
			{
				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == KEY_ON )	//按键确实按下了
				{																	//这里双击不能按太快，会识别成单击
					key_step++;														//进入下一步，需要等松手才能释放状态
					count_time = 0;
				}
				else																//250ms内无按键按下，此时是单击的状态
				{
					if(count_time>65)
					{
						key_step = 0;				//标志位复位
						count_time = 0;					
						return Click;				//返回单击的状态
					}
				}
			}
			break;
		case 5:
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == KEY_ON )//按键还在按着
			{
				count_time++;
			}
			else								//按键已经松手
			{
//				if(count_time>400)				//这里第二次的单击也可以判断时间的，默认不判断时间，全部都返回双击
//				{
//				}
				count_time = 0;
				key_step = 0;
				return Double_Click;
			}
			break;
		default:break;
	}
	return No_Action;							//无动作
}
/* by codex */
