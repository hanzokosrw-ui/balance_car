
#include "show.h"
#include "TrackModule.h"
#include "control.h"
float Velocity_Left,Velocity_Right;	//车轮速度(mm/s)
// Function: OLED display - OLED显示
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
	switch(truth_value & 0x0F)				// 偏左L/偏右R/居中M
	{
		case 6: case 9:						OLED_ShowString(66,16,"M"); break;	// 居中(0110/1001)
		case 1: case 2: case 3: case 7: case 11:	OLED_ShowString(66,16,"L"); break;	// 偏左
		case 4: case 8: case 10: case 12: case 13: case 14:	OLED_ShowString(66,16,"R"); break;	// 偏右
		default:							OLED_ShowString(66,16,"-"); break;	// 十字/丢线
	}
	
	//===========第三行：模式 M 巡线速度 V 转向差速 D==========//
	OLED_ShowString(0,32,"M:");
	OLED_ShowNumber(14,32,Mode,2,12); // 示教模式10/11需要两位显示
	OLED_ShowString(26,32,"V:");
	if(Move_X<0)			OLED_ShowString(40,32,"-");
	else					OLED_ShowString(40,32,"+");
	OLED_ShowNumber(46,32,myabs((int)Move_X),3,12);
	OLED_ShowString(66,32,"D:");
	if(turn_diff<0)			OLED_ShowString(80,32,"-");
	else					OLED_ShowString(80,32,"+");
	OLED_ShowNumber(86,32,myabs((int)turn_diff),2,12);
	switch(Mode)								// 当前模式名
	{
		case Normal_Mode:				OLED_ShowString(100,32,"NOR"); break;	// 普通
		case ROS_Mode:					OLED_ShowString(100,32,"ROS"); break;
		case Ultrasonic_Avoid_Mode:		OLED_ShowString(100,32,"AVD"); break;	// 避障
		case Ultrasonic_Follow_Mode:	OLED_ShowString(100,32,"FLW"); break;	// 跟随
		case IRDM_Line_Patrol_Mode:		OLED_ShowString(100,32,"TRK"); break;	// 巡线
		case Teach_Record_Mode:         OLED_ShowString(100,32,"REC"); break;
		case Teach_Play_Mode:           OLED_ShowString(100,32,"PLY"); break;
		default:						OLED_ShowNumber(100,32,Mode,2,12); break;
	}

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
// Function: Send data to APP - 向APP发送数据
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

/* by codex */



