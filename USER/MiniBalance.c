#include "stm32f10x.h"
#include "sys.h" 
#include "TrackModule.h"
u8 Ros_Rate = 0;                            //ROS发送数据的频率是10HZ，而中断在5ms一次就会把小车的移动的数据清0，这是为了让ros端控制小车运动的更加顺畅的一个计数参数
u8 Ros_count=0,Ros_send_flag;                    //ROS模式下让ROS模式控制的更流畅的变量
u8 Pick_up_stop=0;                          //检查是否被拿起标志位
int Middle_angle=0;                         //机械中值默认为0
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u16 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2,Target_Velocity=300; //蓝牙遥控
float RC_Velocity,RC_Turn_Velocity;			    //遥控控制的速度
u8 Flag_Stop=1,Flag_Show=0;                 //电机停止标志位和显示标志位  默认停止 显示打开
u8 Mode = IRDM_Line_Patrol_Mode;                    //模式选择，默认红外巡线模式
float Move_X,Move_Z;                        //控制小车避障、跟随时前进的变量，转弯的变量
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //温度变量
int Voltage;                                //电池电压
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
u32 Distance;                               //雷达测距
u8 delay_50,PID_Send; 						//延时和调参相关变量
volatile u8 delay_flag;
float Acceleration_Z;                         //Z轴加速度计  
float Balance_Kp=27000,Balance_Kd=110,Velocity_Kp=400,Velocity_Ki=2,Turn_Kp=3000,Turn_Kd=0,Turn_Amplitude=100;//PID参数（放大100倍）
float Distance_KP =250,Distance_KD =10000;//距离调整PID参数
int main(void)
{ 
  MY_NVIC_PriorityGroupConfig(2);	//设置中断分组
	delay_init();	    	            //延时函数初始化	
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试
	LED_Init();                     //初始化与 LED 连接的硬件接口
	KEY_Init();                     //按键初始化
	BEEP_GPIO_Config();             //蜂鸣器初始化
	MiniBalance_PWM_Init(7199,0);   //初始化PWM 10KHZ与电机硬件接口，用于驱动电机
	TeachRemote_Init();             // 示教缓冲区在串口中断使能前初始化
	uart_init(115200);	            //串口1初始化
	uart3_init(9600);             	//串口3初始化，用于蓝牙模块
	Encoder_Init_TIM8();            //初始化编码器8
	Encoder_Init_TIM4();            //初始化编码器4
	Adc_Init();                     //adc初始化
	IIC_Init();                     //IIC初始化
	OLED_Init();                    //OLED初始化
	TrackModule_Init();             //初始化巡线模块
    Distance_Cap_Init(0XFFFF,71);	//超声波初始化
	MPU6050_initialize();           //MPU6050初始化	
	DMP_Init();                     //初始化DMP 
    MiniBalance_EXTI_Init();	      //MPU6050 5ms定时中断初始化，节省定时器资源，减少cpu负担
	while(1)
	{	
		APP_Show();								//发送数据给APP
		oled_show();          			//显示屏打开
		if(Ros_send_flag==1)		
	    {
			data_transition();
			USART1_SEND();                                    //给ros端发送数据 50ms一次·
			Ros_send_flag=0;
	    }
        delay_flag=1;		
	   while(delay_flag);      	
	}
}

// by codex

