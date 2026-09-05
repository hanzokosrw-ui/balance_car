#include "TrackModule.h"
#include "control.h"
// ===== 可调参数区域 =====
// 转向角度参数
float Turn90Angle  = 80;   // 直角弯转向参数
float TurnMaxAngle = 65;   // 大弯道转向参数
float TurnMidAngle = 40;   // 中等转向参数（丢线时使用）
float TurnMinAngle = 15;   // 微调转向参数
// 速度参数
float BaseSpeed = 300;      // 基础巡线速度（直行时的速度）
float ForwardLimit = 400;		//前行限制(转向大于该值限制其前进)
// ===== 传感器状态定义--识别到黑线时为1 =====
typedef enum {
    STATE_END         = 0,    // 0000 - 十字路口
    STATE_LEFT_90_A     = 1,    // 0001 - 左直角
    STATE_LEFT_INNER    = 2,    // 0010 - 仅左内传感器（偏左微调）
    
	STATE_LEFT_90_B		= 3,	// 0011
    STATE_RIGHT_INNER   = 4,    // 0100 - 仅右内传感器（偏右微调）
    STATE_STRAIGHT_MID  = 6,    // 0110 - 内侧两传感器对称（直行）
    STATE_LEFT_BIG      = 7,    // 0111 - 左大弯
    STATE_RIGHT_90_A    = 8,  	// 1000 - 右直角弯
    STATE_STRAIGHT      = 9,    // 1001 - 直行
    STATE_DIAG_RIGHT    = 10,   // 1010 - DH1+DH3 对角偏右
    STATE_LEFT_SMALL    = 11,   // 1011 - 左微调
    STATE_RIGHT_90_B    = 12,	// 1100
    STATE_RIGHT_SMALL   = 13,   // 1101 - 右微调
    STATE_RIGHT_BIG     = 14,   // 1110 - 右大弯
    STATE_LOST          = 15    // 1111 - 丢线
} SensorState_t;

float base_speed_mm = 0;// 基础速度（mm/s）
float turn_diff = 0;    // 转向差速

#define TIMED_TURN_TICKS 20u  /* 可调：每个计数周期为10 ms */

EightTrackState_t eight_track_state = EIGHT_TRACK_IDLE;
static u16 timed_turn_timer = 0;
static u8 eight_track_segment_flag = 0; /* 0=未执行，1=第一段完成，2=两段完成 */

// ===== 巡线功能函数（输出两电机目标速度） =====
void IRDM_line_inspection(void)
{
    static int last_state = 0;// 记录上一次的状态
    int timed_turn_diff = 0;

    // 读取传感器状态：4个传感器组合值
    int sensor_state = (DH1 << 3) | (DH2 << 2) | (DH3 << 1) | DH4;

    /* 第一段：1000/1100 -> 定时原地右转 -> 等待0000 -> 左转。 */
    if (eight_track_state == EIGHT_TRACK_IDLE && eight_track_segment_flag == 0 &&
        (sensor_state == STATE_RIGHT_90_A || sensor_state == STATE_RIGHT_90_B))
    {
        eight_track_state = EIGHT_TRACK_FIRST_TURN_RIGHT;
        timed_turn_timer = 0;
    }

    if (eight_track_state == EIGHT_TRACK_FIRST_TURN_RIGHT)
    {
        timed_turn_diff = -80;
        if (++timed_turn_timer >= TIMED_TURN_TICKS)
        {
            timed_turn_timer = 0;
            eight_track_state = EIGHT_TRACK_FIRST_WAIT_END;
            Velocity_Request_Integral_Scale(0.2f);
        }
    }
    else if (eight_track_state == EIGHT_TRACK_FIRST_WAIT_END)
    {
        if (sensor_state == STATE_END)
        {
            eight_track_state = EIGHT_TRACK_FIRST_TURN_LEFT;
            timed_turn_timer = 0;
        }
    }

    if (eight_track_state == EIGHT_TRACK_FIRST_TURN_LEFT)
    {
        timed_turn_diff = 80;
        if (++timed_turn_timer >= TIMED_TURN_TICKS)
        {
            timed_turn_timer = 0;
            eight_track_state = EIGHT_TRACK_WAIT_LEFT_ANGLE;
            eight_track_segment_flag = 1;
            Velocity_Request_Integral_Scale(0.2f);
        }
    }

    /* 第二段：0001/0011 -> 定时原地左转 -> 等待0000 -> 右转。 */
    if (eight_track_state == EIGHT_TRACK_WAIT_LEFT_ANGLE && eight_track_segment_flag == 1 &&
        (sensor_state == STATE_LEFT_90_A || sensor_state == STATE_LEFT_90_B))
    {
        eight_track_state = EIGHT_TRACK_SECOND_TURN_LEFT;
        timed_turn_timer = 0;
    }

    if (eight_track_state == EIGHT_TRACK_SECOND_TURN_LEFT)
    {
        timed_turn_diff = 80;
        if (++timed_turn_timer >= TIMED_TURN_TICKS)
        {
            timed_turn_timer = 0;
            eight_track_state = EIGHT_TRACK_SECOND_WAIT_END;
            Velocity_Request_Integral_Scale(0.2f);
        }
    }
    else if (eight_track_state == EIGHT_TRACK_SECOND_WAIT_END)
    {
        if (sensor_state == STATE_END)
        {
            eight_track_state = EIGHT_TRACK_SECOND_TURN_RIGHT;
            timed_turn_timer = 0;
        }
    }

    if (eight_track_state == EIGHT_TRACK_SECOND_TURN_RIGHT)
    {
        timed_turn_diff = -80;
        if (++timed_turn_timer >= TIMED_TURN_TICKS)
        {
            timed_turn_timer = 0;
            eight_track_state = EIGHT_TRACK_IDLE;
            eight_track_segment_flag = 2;
            Velocity_Request_Integral_Scale(0.2f);
        }
    }
        // ===== 状态判断：设置转向差速 =====
    switch (sensor_state)
    {
       case STATE_END:// 终点停车：直接切回普通模式
			turn_diff = 0;
			if(eight_track_state == EIGHT_TRACK_IDLE && timed_turn_diff == 0)
				Mode = Normal_Mode;		//状态机外检测到0000，退出巡线并停车
            break;
        case STATE_LEFT_INNER: // 仅左内传感器，偏左微调
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_INNER: // 仅右内传感器，偏右微调
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT_MID: // 内侧两传感器对称，直行
            turn_diff = 0;
            break;
        case STATE_DIAG_RIGHT: // DH1+DH3 对角偏右
            //turn_diff = -TurnMinAngle;
            break;
        case STATE_LEFT_90_A: // 左直角弯
		case STATE_LEFT_90_B: // 左直角弯
            //turn_diff = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // 右直角弯
		case STATE_RIGHT_90_B: // 右直角弯
            //turn_diff = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://左大弯
            //turn_diff = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG://右大弯
            turn_diff = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL://左微调
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL://右微调
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT://直行
            turn_diff = 0;
            break;
        case STATE_LOST://丢线处理
            // if (last_state == STATE_LEFT_SMALL) turn_diff = TurnMidAngle;//继续左转
			// else if (last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;//继续右转
			// else if(last_state == STATE_LEFT_BIG ) turn_diff = TurnMaxAngle;//继续左转
			// else if(last_state == STATE_RIGHT_BIG ) turn_diff = -TurnMaxAngle;//继续右转
            turn_diff = last_state;
            break;
        default: // 未定义状态，直行
            turn_diff = 0;
            break;
    }
	//保存传感器状态
	if(sensor_state!=STATE_LOST)
	{
		last_state=turn_diff;
	}
	// 转向速度越大，基础速度越低
	if(fabs(turn_diff)<ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
	}
	else base_speed_mm=0;
	if(timed_turn_diff != 0)
	{
		turn_diff = timed_turn_diff;
		base_speed_mm = 0;
	}
	else if(sensor_state == STATE_END && eight_track_state == EIGHT_TRACK_IDLE)
	{
		base_speed_mm = 0;
	}
        // ===== Output: base_speed_mm(line speed) & turn_diff(turn differential) =====
}

void TrackModule_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    eight_track_state = EIGHT_TRACK_IDLE;
    timed_turn_timer = 0;
    eight_track_segment_flag = 0;

    // 使能传感器引脚时钟（引脚宏在 TrackModule.h 中配置）
    RCC_APB2PeriphClockCmd(TRACK_GPIO_CLK1 | TRACK_GPIO_CLK2, ENABLE);
    // 配置 GPIOC 引脚（PC8/PC4/PC9）为输入下拉模式
    GPIO_InitStructure.GPIO_Pin = TRACK_GPIO_PIN1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 输入下拉模式
    GPIO_Init(TRACK_GPIO_PORT1, &GPIO_InitStructure);
    // 配置 GPIOB 引脚（PB8）为输入下拉模式
    GPIO_InitStructure.GPIO_Pin = TRACK_GPIO_PIN2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 输入下拉模式
    GPIO_Init(TRACK_GPIO_PORT2, &GPIO_InitStructure);
}

// Function: IRDM_Line_Seen - 判断是否检测到线（绕障后找线用）
u8 IRDM_Line_Seen(void)   // 1=检测到线，0=丢线/终点
{
	int s = (DH1<<3)|(DH2<<2)|(DH3<<1)|DH4;
	return (s != STATE_LOST && s != STATE_END);
}

// Function: EightTrack_IsIdle - 特殊路况（8字轨道）状态机是否空闲（避障触发互斥用）
u8 EightTrack_IsIdle(void)
{
	return eight_track_state == EIGHT_TRACK_IDLE;
}

/* by codex */
















