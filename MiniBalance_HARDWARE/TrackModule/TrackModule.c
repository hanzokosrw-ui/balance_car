#include "TrackModule.h"
/*=============================================================================*
 * 可调参数区域																   *
 *=============================================================================*/
// 转向角度参数
float Turn90Angle  = 80;   // 直角弯转向参数
float TurnMaxAngle = 65;   // 大弯道转向参数
float TurnMidAngle = 40;   // 中等转向参数（丢线时使用）
float TurnMinAngle = 15;   // 微调转向参数
// 速度参数
float BaseSpeed = 0;      // 基础巡线速度（直行时的速度）
float ForwardLimit = 50;		//前行限制(转向大于该值限制其前进)
/*=============================================================================*
 * 传感器状态定义--识别到黑线时为1											   *
 *=============================================================================*/
typedef enum {
    STATE_CROSS         = 0,    // 0000 - 十字路口
    STATE_LEFT_90_A     = 1,    // 0001 - 左直角弯
	STATE_LEFT_90_B		= 3,	// 0011
    STATE_RIGHT_90_A    = 8,  	// 1000 - 右直角弯
	STATE_RIGHT_90_B    = 12,	// 1100
    STATE_LEFT_BIG      = 7,    // 0111 - 左大弯
    STATE_RIGHT_BIG     = 14,   // 1110 - 右大弯
    STATE_LEFT_SMALL    = 11,   // 1011 - 左微调
    STATE_RIGHT_SMALL   = 13,   // 1101 - 右微调
    STATE_STRAIGHT      = 9,    // 1001 - 直行
    STATE_LOST          = 15    // 1111 - 丢线
} SensorState_t;

float base_speed_mm = 0;// 基础速度（mm/s）
float turn_diff = 0;    // 转向差速

/*=============================================================================*
 * 巡线功能函数（输出两电机目标速度）											   *
 *=============================================================================*/
void IRDM_line_inspection(void)
{
    static int last_state = 0;// 记录上一次的状态

    // 读取传感器状态：4个传感器组合值
    int sensor_state = (DH1 << 3) | (DH2 << 2) | (DH3 << 1) | DH4;
    /*=========================================================================*
     * 状态判断：设置转向差速												   *
     *=========================================================================*/
    switch (sensor_state)
    {
       case STATE_CROSS:// 交叉路口处理
			turn_diff = 0;
            break;
        case STATE_LEFT_90_A: // 左直角弯
		case STATE_LEFT_90_B: // 左直角弯
            turn_diff = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // 右直角弯
		case STATE_RIGHT_90_B: // 右直角弯
            turn_diff = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://左大弯
            turn_diff = TurnMaxAngle;
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
            turn_diff = 0;//暂时先直行
            break;
        default: // 未定义状态，直行
            turn_diff = 0;
            break;
    }
	//保存传感器状态
	if(sensor_state!=STATE_LOST)
	{
		last_state=sensor_state;
	}
	// 转向速度越大，基础速度越低
	if(fabs(turn_diff)<ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
	}
	else base_speed_mm=0;
    /*========================================================================*
     * Output: base_speed_mm(line speed) & turn_diff(turn differential)        *
     * Used by IRDM_Mode()/IRDM_turn() in control.c                            *
     *=========================================================================*/
}

void TrackModule_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

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
















