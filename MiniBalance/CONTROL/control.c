#include "control.h"	
#include "TrackModule.h"	
short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
// ===== 绕障参数（模式7执行绕障；模式9巡线遇障自动交接）=====
#define AVOID_TRIG_DIST    250          // 触发距离mm
#define AVOID_TRIG_CNT     5            // 连续判定次数（50ms）防误触发
#define AVOID_TURN_ANGLE   90           // 每次转弯角度（度）
#define AVOID_GYRO_SCALE   16.4f        // 陀螺仪灵敏度 LSB/(°/s)
#define AVOID_TURN_MAX_MS  50           // 单次转弯超时（500ms）兜底
#define AVOID_FWD1_MM       30          // 前行1里程（mm，原100ms@300mm/s）
#define AVOID_FWD2_MM      (AVOID_TRIG_DIST*0.6)          // 前行2里程（mm，须大于障碍长度+余量）
#define AVOID_FWD3_MAX_MM  (AVOID_FWD1_MM*100)          // 前行3找线最大里程（mm）兜底

// ===== 特殊路况（8字轨道）定时转弯后的速度积分缩放（队友：防止转弯后猛冲）=====
static float velocity_integral_scale_request = 1.0f;

void Velocity_Request_Integral_Scale(float factor)
{
	if(factor < 0) factor = 0;
	if(factor > 1) factor = 1;
	velocity_integral_scale_request = factor;
}

// ===== 绕障状态 =====
#define AVOID_IDLE      0               // 正常巡线
#define AVOID_TURN_R1   1               // 右转90°
#define AVOID_FWD1      2               // 前行
#define AVOID_TURN_L1   3               // 左转90°
#define AVOID_FWD2      4               // 前行
#define AVOID_TURN_L2   5               // 左转90°
#define AVOID_FWD3      6               // 前行直到找到线
#define AVOID_TURN_R2   7               // 右转90°回正

u8  avoid_state = AVOID_IDLE;           // 当前绕障状态
u8  avoid_cnt   = 0;                    // 连续触发计数
u16 avoid_timer = 0;                    // 阶段计时（10ms，转弯超时用）
float avoid_angle = 0;                  // 转弯积分角度（度）
float avoid_mm   = 0;                   // 前行累计里程（mm，编码器反馈）
u8   avoid_ret   = 0;                   // 绕障来源：1=巡线交接（绕完回巡线），0=独立模式7
// Function: Control function - 所有的控制代码都在这里面
int EXTI9_5_IRQHandler(void) 
{ 
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//电压测量相关变量
	static u8 Flag_Target;																//控制函数相关变量，提供10ms基准
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//平衡环PWM变量，速度环PWM变量，转向环PWM变
	if(INT==0)		
	{   
		EXTI->PR=1<<9;                           					  //清除中断标志位   
		Encoder_Left=Read_Encoder(4);            					  //读取左轮编码器的值，前进为正，后退为负
		Encoder_Right=-Read_Encoder(8);           					//读取右轮编码器的值，前进为正，后退为负
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果

//																												//左轮A相接TIM4_CH1,右轮A相接TIM8_CH1,故这里两个编码器的极性不相同
		Mode_Choose();                                      //模式的选择
		if(Mode == ROS_Mode) TeachRemote_SetMode(Normal_Mode); //禁止任何异常路径进入ROS模式
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//编码器读数转速度（mm/s）
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0; 		//给主函数提供50ms的精准延时，示波器需要50ms高精度延时
        }
		if(++Ros_count == 10)
		{
			Ros_send_flag=1;
			Ros_count=0;
		}
		if(Flag_Target==1)                        					//10ms控制一次
		{		
			Voltage_Temp=Get_battery_volt();		    					//读取电池电压		
			Voltage_Count++;                       						//平均值计数器
			Voltage_All+=Voltage_Temp;              					//多次采样累积
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值		
			return 0;	                                               
		}                                         					//10ms控制一次
		TeachRemote_Tick(Turn_Off(Angle_Balance,Voltage)==0); // 10ms记录/回放，不阻塞平衡环
		if(Mode==Ultrasonic_Avoid_Mode||Mode==Ultrasonic_Follow_Mode||Mode==IRDM_Line_Patrol_Mode)		
	       Read_Distane();                                  //超声波读取距离   
		Select_Zhongzhi();                                  //机械中值选择
		IRDM_Mode();                                        //红外循迹模式
		if(Mode==Ultrasonic_Avoid_Mode || Mode==IRDM_Line_Patrol_Mode)  Avoid_State_Machine();  //模式7绕障宿主/模式9巡线，均推进绕障状态机
		else { avoid_state = AVOID_IDLE; avoid_cnt = 0; avoid_ret = 0; }                       //其他模式复位
		if(Mode==Normal_Mode)	Led_Flash(100);             //LED闪烁;常规模式 1s改变一次指示灯的状态	
		else Led_Flash(0);                                  //LED常亮;其余模式
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //速度环PID控制	记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		if(avoid_state != AVOID_IDLE)  Turn_Pwm = Avoid_Turn();                     //绕障中：定角转向（模式7/9通用）
		else if(Mode ==IRDM_Line_Patrol_Mode)  Turn_Pwm = IRDM_turn(turn_diff,Gyro_Turn);  //巡线转向
		else  Turn_Pwm = Turn(Gyro_Turn);                                           //其余（含模式7待机）：遥控转向

		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;         //计算左轮电机最终PWM
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;        //计算右轮电机最终PWM
																												  //PWM值正数使小车前进，负数使小车后退
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);		  	//PWM限幅

		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被拿起
			Pick_up_stop=1;	                           					//如果被拿起就关闭电机,当小车被拿起时，需要关闭key2或者放下推一下就可以正常运行
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被放下
			Pick_up_stop=0;	                   		              //如果被放下就启动电机
		if(Turn_Off(Angle_Balance,Voltage)==0)     					  //如果不存在异常
			Set_Pwm(Motor_Left,Motor_Right);         					  //赋值给PWM寄存器  
		else
			TeachRemote_SafetyStop(); // 提起/倾倒等保护触发时终止示教
	 }       	
	 return 0;	  
} 

// Function: Vertical PD control - 直立PD控制	
int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值 和机械相关
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

// Function: Speed PI control - 速度控制PWM
int Velocity(int encoder_left,int encoder_right)
{  
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral;
	  //================遥控前进后退部分====================// 
		if(Flag_front==1)    	Movement=Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;	  //收到前进信号
		else if(Flag_back==1)	Movement=-Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;  //收到后退信号
	  else  Movement=Move_X/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;//将给定速度转化为电机的编码器读数单位;
          //Movement=Move_X/周长/编码器读取频率*频数*减速比*精度*2;	
	  if(Movement>2400)  Movement=2400;                     //蓝牙遥控速度的限制，避免破坏小车的平衡
	
	   //=============超声波功能（跟随/避障）==================// 
	  if(Mode==Ultrasonic_Follow_Mode&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //跟随
			 Movement=Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;
		if(Mode==Ultrasonic_Follow_Mode&&Distance<200&&Flag_Left!=1&&Flag_Right!=1) 
			 Movement=-Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;
		//（模式7旧“倒车避障”已删：模式7现为绕障宿主，倒车会抵消触发计数，由绕障状态机接管）
		
		//===========绕障：转弯阶段停车，前行阶段前进===========//
		if(avoid_state != AVOID_IDLE)
		{
			if(avoid_state==AVOID_FWD1||avoid_state==AVOID_FWD2||avoid_state==AVOID_FWD3)
				Movement=Target_Velocity/Perimeter/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision*2;  //前行绕障
			else
				Movement=0;          //转弯阶段先停车
		}
		
   //================速度PI控制器=====================//	
		Encoder_Least =0-(encoder_left+encoder_right);                    //获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和） 
		Encoder_bias *= 0.84;		                                          //一阶低通滤波器       
		Encoder_bias += Encoder_Least*0.16;	                              //一阶低通滤波器，减缓速度变化 
		Encoder_Integral +=Encoder_bias;                                  //积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //接收遥控器数据，控制前进后退
		if(velocity_integral_scale_request < 1.0f)
		{
			Encoder_Integral *= velocity_integral_scale_request;
			velocity_integral_scale_request = 1.0f;
		}
		if(Encoder_Integral>380000)  	Encoder_Integral=380000;             //积分限幅
		if(Encoder_Integral<-380000)	  Encoder_Integral=-380000;            //积分限幅	
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //速度控制
	  if(Mode == ROS_Mode)
		{ 
			if(++Ros_Rate>=100) Ros_Rate=0,Move_X=0;//如果ros端200ms内没有发送数据过来，Move_Z置0；这是为了ros端控制小车的更加顺畅
		}
		else Move_X	=0;	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//电机关闭后清除积分
	  return velocity;
}
// Function: Turn control - 转向控制
int Turn(float gyro)
{
	 static float Turn_Target,turn;
	 float Kp=Turn_Kp,Kd;			//修改转向速度，请修改Turn_Amplitude即可（全局，与巡线共用）
	//===================遥控左右旋转部分=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
	 else Turn_Target=0;
	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;        
	 else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  //===================转向PD控制器=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100+Move_Z;//结合Z轴陀螺仪进行PD控制
     if(Mode == ROS_Mode)
	 {
		 if(++Ros_Rate>=40) Ros_Rate=0,Move_Z=0;//如果ros端200ms没有发送数据过来，Move_Z置0；这是为了ros端控制小车的更加顺畅
	 }
	 else Move_Z=0;
	 return turn;								 				 //转向环PWM右转为正，左转为负
}

// Function: Assign to PWM register - 赋值给PWM寄存器
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)		
	{
		PWMA_IN1=7200;
		PWMA_IN2=7200-motor_left;//左轮前进
	}		
	else 
	{
		PWMA_IN1=7200+motor_left;
		PWMA_IN2=7200;
	} //左轮后退
  if(motor_right>0)			
	{
		PWMB_IN1=7200-motor_right;
		PWMB_IN2=7200;
	}		//右轮前进
	else 	        			  
	{
		PWMB_IN1=7200;
		PWMB_IN2=7200+motor_right;
	}//右轮后退
}
// Function: PWM limiting range - 限制PWM赋值
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}


// Function: If abnormal, turn off the motor - 异常关闭电机	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	Flag_Stop = KEY2_STATE;                             
	if(KEY2_STATE==1) Pick_up_stop=0;                  //key2关闭，Pick_up_stop恢复为0
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000||Pick_up_stop==1)//电池电压低于11.1V关闭电机
	{	                                                 //倾角大于40度关闭电机
		temp=1;                                          //Flag_Stop置1，即单击控制关闭电机
		PWMA_IN1=0;                                      //Pick_up_stop置1，即小车基本静止，在0度左右拿起小车      
		PWMA_IN2=0;
		PWMB_IN1=0;
		PWMB_IN2=0;
	}
	else
		temp=0;
	return temp;			
}
	
// Function: Get angle - 获取角度	
void Get_Angle(u8 way)
{ 
  float gyro_x,gyro_y;
	Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
		Read_DMP();                      	 //读取加速度、角速度、倾角
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn=gyro[2];                 //更新转向角速度
		Acceleration_Z=accel[2];           //更新Z轴加速度计
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
//		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
//		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
//		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
//		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //计算倾角，转换单位为度	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //计算倾角，转换单位为度
		gyro_x=Gyro_X/16.4;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
		gyro_y=Gyro_Y/16.4;                              //陀螺仪量程转换	
		if(Way_Angle==2)		  	
		{
			 Pitch = -Kalman_Filter_x(Accel_Angle_x,gyro_x);//卡尔曼滤波
			 Roll = -Kalman_Filter_y(Accel_Angle_y,gyro_y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//互补滤波
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计	
	}
}
// Function: Absolute value function - 绝对值函数	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
// Function: Check whether the car is picked up - 检测小车是否被拿起
// ===== 函数功能：检测小车是否被拿起 =====
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	static u16 flag,count0,count1,count2;
	if(flag==0)                                                                   //第一步
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<150)                         //条件1，小车接近静止
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			   flag=1,count0=0; 
	 } 
	 if(flag==1)                                                                  //进入第二步
	 {
			if(++count1>200)       count1=0,flag=0;                                 //超时不再等待2000ms
			if(Acceleration>30000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //条件2，小车是在0度附近被拿起
			{
				flag = 2;
			}
		}
	 if(flag == 2)
	 {
		  if(++count2>100)       count2=0,flag=0;                    //超时不再等待1000ms
		  if(myabs(encoder_left+encoder_right)>3000)                   //条件3，小车的轮胎因为正反馈达到最大的转速
			{
				flag=0;
				return 1;                                                //检测到小车被拿起
			}
	 }		
	return 0;
}
// Function: Check whether the car is lowered - 检测小车是否被放下
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Pick_up_stop==0)                     //防止误检      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //条件1，小车是在0度附近的
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<100&&encoder_right<100) //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				flag=0;
				return 1;                         //检测到小车被放下
			}
	 }
	return 0;
}
// Function: Encoder reading is converted to speed (mm/s) - 编码器读数转换为速度（mm/s）
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//电机转速  转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//求出编码器速度=转速*周长
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//求出编码器速度=转速*周长
}




// Function: Select_Zhongzhi - 小车机械中值的选择
void Select_Zhongzhi(void)                   //机械中值选择，避免安装巡线装备时小车往前冲的现象
{
	if(Mode == IRDM_Line_Patrol_Mode || (Mode==Ultrasonic_Avoid_Mode && avoid_state != AVOID_IDLE))
		Middle_angle = -2;                   //巡线/绕障同用巡线机械中值，避免9→7交接时平衡突变
	else   Middle_angle = 1;
}

// Function: IRDM_Mode - 红外循迹模式运行（在10ms控制中断中调用）
void IRDM_Mode(void)
{
	if(Mode == IRDM_Line_Patrol_Mode && Flag_Left != 1 && Flag_Right != 1)
	{
		IRDM_line_inspection();              //读取传感器，更新 base_speed_mm / turn_diff
		Move_X = base_speed_mm;              //巡线速度（mm/s）
	}
}

// Function: IRDM_turn - 红外循迹模式转向控制
int IRDM_turn(float turn_diff, float gyro)
{
	float Turn;

	// 折算：轮速差→遥控转向幅度（右转为正，左转为负），再套用遥控 PD 结构
	// 注：若巡线转向方向相反，改这里正负号即可（当前已按实测方向修正）
	Turn = -((turn_diff / Turn90Angle) * Turn_Amplitude * Turn_Kp / 100
	       - gyro * Turn_Kd / 100);

	return (int)Turn;
}

// ===== 绕障转向命令（右转为正，左转为负）=====
int Avoid_Turn(void)
{
	if(avoid_state==AVOID_TURN_R1 || avoid_state==AVOID_TURN_R2) return  Turn_Amplitude*Turn_Kp/200;  // 右转
	if(avoid_state==AVOID_TURN_L1 || avoid_state==AVOID_TURN_L2) return -Turn_Amplitude*Turn_Kp/200;  // 左转
	return 0;
}

// ===== 绕障状态机（10ms调用一次；模式7执行，模式9自动交接/回巡线）=====
void Avoid_State_Machine(void)
{
	switch(avoid_state)
	{
		case AVOID_IDLE:                                 // 待机：连续N次测到障碍才触发绕障
			if(Distance>20 && Distance<AVOID_TRIG_DIST)   // 避障优先级最高：8字处理中也可触发；避障与巡线互不干扰（巡线状态冻结，绕完续跑）
			{
				if(++avoid_cnt >= AVOID_TRIG_CNT)
				{
					avoid_cnt=0; avoid_state=AVOID_TURN_R1; avoid_angle=0; avoid_timer=0; avoid_mm=0;
					if(Mode==IRDM_Line_Patrol_Mode){ Mode=Ultrasonic_Avoid_Mode; avoid_ret=1; }  //巡线遇障→借道模式7，绕完回巡线
					else avoid_ret=0;                                                              //独立模式7：绕完停在模式7
				}
			}
			else avoid_cnt=0;
			break;
		case AVOID_TURN_R1:                              // 先停车，右转90°
		case AVOID_TURN_L1:                              // 左转90°
		case AVOID_TURN_L2:                              // 左转90°
		case AVOID_TURN_R2:                              // 右转90°回正
		{
			float target = AVOID_TURN_ANGLE;
			if(avoid_state==AVOID_TURN_L1 || avoid_state==AVOID_TURN_L2) target = -AVOID_TURN_ANGLE;
			avoid_angle += (Gyro_Turn/AVOID_GYRO_SCALE)*0.01f;          // 积分实际转角
			if((target>0 ? avoid_angle>=target : avoid_angle<=target) || ++avoid_timer>=AVOID_TURN_MAX_MS)
			{
				avoid_angle=0; avoid_timer=0; avoid_mm=0;
				if(avoid_state==AVOID_TURN_R1)      avoid_state=AVOID_FWD1;
				else if(avoid_state==AVOID_TURN_L1) avoid_state=AVOID_FWD2;
				else if(avoid_state==AVOID_TURN_L2) avoid_state=AVOID_FWD3;
				else { avoid_state=AVOID_IDLE; avoid_cnt=0;                                     // 绕障完成
					   if(Mode==Ultrasonic_Avoid_Mode && avoid_ret) Mode=IRDM_Line_Patrol_Mode;   // 巡线交接→回巡线
					   avoid_ret=0; }
			}
		}
			break;
		case AVOID_FWD1:                                 // 前行过障碍：按实际里程切下一状态
			avoid_mm += (Velocity_Left + Velocity_Right) * 0.005f;  // 轮速均值(mm/s)×10ms→mm
			if(avoid_mm >= AVOID_FWD1_MM)
			{ avoid_state=AVOID_TURN_L1; avoid_angle=0; avoid_timer=0; avoid_mm=0; }
			break;
		case AVOID_FWD2:                                 // 前行
			avoid_mm += (Velocity_Left + Velocity_Right) * 0.005f;
			if(avoid_mm >= AVOID_FWD2_MM)
			{ avoid_state=AVOID_TURN_L2; avoid_angle=0; avoid_timer=0; avoid_mm=0; }
			break;
		case AVOID_FWD3:                                 // 前行直到找到线（或里程兜底）
			avoid_mm += (Velocity_Left + Velocity_Right) * 0.005f;
			if(IRDM_Line_Seen() || avoid_mm >= AVOID_FWD3_MAX_MM)
			{ avoid_state=AVOID_TURN_R2; avoid_angle=0; avoid_timer=0; avoid_mm=0; }
			break;
	}
}

/* by codex */




