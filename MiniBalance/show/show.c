/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/
#include "show.h"
#include "TrackModule.h"
float Velocity_Left,Velocity_Right;	//车轮速度(mm/s)
/**************************************************************************
Function: OLED display
Input   : none
Output  : none
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	u8 truth_value;

	 memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM清零但不立即刷新，防止花屏

	truth_value = ((DH1?1:0)<<3) | ((DH2?1:0)<<2) | ((DH3?1:0)<<1) | (DH4?1:0);

	//===========第一行：平衡角 A 与角速度 G=================//
	OLED_ShowString(0,0,"A:");
	if(Angle_Balance<0)		OLED_ShowString(14,0,"-");
	else					OLED_ShowString(14,0,"+");
	OLED_ShowNumber(20,0,myabs((int)Angle_Balance),3,12);
	OLED_ShowString(40,0,"G:");
	if(Gyro_Balance<0)		OLED_ShowString(54,0,"-");
	else					OLED_ShowString(54,0,"+");
	OLED_ShowNumber(60,0,myabs((int)Gyro_Balance),4,12);

	//===========第二行：传感器 TR 与状态值 ST================//
	OLED_ShowString(0,16,"TR:");
	OLED_ShowNumber(24,16,(truth_value >> 3) & 0x01,1,12);
	OLED_ShowNumber(30,16,(truth_value >> 2) & 0x01,1,12);
	OLED_ShowNumber(36,16,(truth_value >> 1) & 0x01,1,12);
	OLED_ShowNumber(42,16,truth_value & 0x01,1,12);
	OLED_ShowString(52,16,"ST:");
	OLED_ShowNumber(68,16,truth_value,2,12);
	
	//===========第三行：模式 M 巡线速度 V 转向差速 D==========//
	OLED_ShowString(0,32,"M:");
	OLED_ShowNumber(16,32,Mode,1,12);
	OLED_ShowString(26,32,"V:");
	if(Move_X<0)			OLED_ShowString(40,32,"-");
	else					OLED_ShowString(40,32,"+");
	OLED_ShowNumber(46,32,myabs((int)Move_X),3,12);
	OLED_ShowString(66,32,"D:");
	if(turn_diff<0)			OLED_ShowString(80,32,"-");
	else					OLED_ShowString(80,32,"+");
	OLED_ShowNumber(86,32,myabs((int)turn_diff),2,12);

	//===========第四行：左右电机 PWM L/R=====================//
	OLED_ShowString(0,48,"L:");
	if(Motor_Left<0)		OLED_ShowString(14,48,"-");
	else					OLED_ShowString(14,48,"+");
	OLED_ShowNumber(20,48,myabs((int)Motor_Left),4,12);
	OLED_ShowString(46,48,"R:");
	if(Motor_Right<0)		OLED_ShowString(60,48,"-");
	else					OLED_ShowString(60,48,"+");
	OLED_ShowNumber(66,48,myabs((int)Motor_Right),4,12);

		
		//=============刷新=======================//
		OLED_Refresh_Gram();	
}
/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
  static u8 flag;
	int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1110)*2/3;		if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //对电压数据进行处理
	Encoder_Right_Show=Velocity_Right*1.1; if(Encoder_Right_Show<0) Encoder_Right_Show=-Encoder_Right_Show;			  //对编码器数据就行数据处理便于图形化
	Encoder_Left_Show=Velocity_Left*1.1;  if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
	flag=!flag;
	if(PID_Send==1)			//发送PID参数,在APP调参界面显示
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Target_Velocity,(int)Balance_Kp,(int)Balance_Kd,(int)Velocity_Kp,(int)Velocity_Ki,(int)Turn_Kp,(int)Turn_Kd,(int)Distance_KP,(int)Distance_KD);//打印到APP上面	
		PID_Send=0;	
	}	
   else	if(flag==0)		// 发送电池电压，速度，角度等参数，在APP首页显示
	 {
		 printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)Angle_Balance); //打印到APP上面
	 }
		
	 else								//发送小车姿态角，在波形界面显示
	   printf("{B%d:%d:%d}$",(int)Gyro_Balance,(int)Gyro_Balance,(int)Gyro_Balance); //x，y，z轴角度 在APP上面显示波形
																													//可按格式自行增加显示波形，最多可显示五个
}



