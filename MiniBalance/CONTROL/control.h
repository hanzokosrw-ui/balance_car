
#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "TeachRemote.h"

#define PI 3.14159265							//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  500.0 	//编码器精度 500线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm


#define DIFFERENCE 100



//小车各模式定义
#define Normal_Mode							0
#define ROS_Mode				            6
#define Ultrasonic_Avoid_Mode               7
#define Ultrasonic_Follow_Mode              8
#define IRDM_Line_Patrol_Mode               9   //红外循迹模式
#define Teach_Record_Mode                  10  // 遥控记录：R
#define Teach_Play_Mode                    11  // 示教回放：P

int EXTI9_5_IRQHandler(void);
int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left,int motor_right);
void Limit_Pwm(void);
int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle, int voltage);
void Middle_angle_Check(void);
void Get_Angle(u8 way);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
void Choose(int encoder_left,int encoder_right);
void Select_Zhongzhi(void);
void IRDM_Mode(void);
int IRDM_turn(float turn_diff,float gyro);
void Avoid_State_Machine(void);
extern short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;

#endif

/* by codex */

