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
    STATE_T         = 0,    // 0000 - T字路口
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
// ===== 特殊状态序列：全4路特殊态按序匹配（直角弯/停车）；内两路只巡线 =====
typedef struct {
    u8    state;   // 期待匹配的全4位传感器状态（黑=1）
    float turn;    // 命中后施加的转向差速（左+右-）；0=不强加
    u8    stop;    // 1=命中后切普通模式停车（终点步）
} SeqStep_t;

SeqStep_t sensor_state_table[] = {                 // 赛道顺序，实测后按需修改
    {STATE_RIGHT_BIG, -Turn90Angle, 0},   // 命中 RIGHT_BIG → 强加 -Turn90Angle
    {STATE_T,          Turn90Angle,  0},  // 命中 T(0000)   → 强加 +Turn90Angle
    {STATE_LEFT_BIG,   Turn90Angle,  0},  // 命中 LEFT_BIG  → 强加 +Turn90Angle
    {STATE_T,          -Turn90Angle, 0},  // 命中 T(0000)   → 强加 -Turn90Angle
    {STATE_T,          0,            1},  // 命中 T(0000)   → 终点停车(切普通模式)
};
#define SEQ_STEP_NUM        (sizeof(sensor_state_table)/sizeof(SeqStep_t))
#define SEQ_COOL_TICKS      40            // 命中后屏蔽：40×10ms=400ms
#define SEQ_HOLD_MAX_TICKS  500           // 强制转向兜底上限：5s（防一直硬拐找不到下一标记）

static u8    seq_step = 0;        // 当前期待步索引
static u8    seq_cool = 0;        // 命中后屏蔽计数
static u8    seq_hold = 0;        // 1=强制转向段（turn_diff 压成指令值）
static u16   seq_hold_ticks = 0;  // 强制转向持续计数
static float seq_hold_turn = 0;   // 强制转向值
float base_speed_mm = 0;// 基础速度（mm/s）
float turn_diff = 0;    // 转向差速

// ===== 巡线功能函数（仅内两路 DH2/DH3 巡线；外两路/大片特殊态由特殊状态机处理） =====
void IRDM_line_inspection(void)
{
    static int last_turn = 0;               // 两内都离线时沿用上次转向

    // 只取内两路：DH2=右内(bit1)，DH3=左内(bit0) → inner: 3=都在线 2=仅DH2 1=仅DH3 0=都离线
    int inner = ((DH2?1:0)<<1) | (DH3?1:0);

    switch(inner)
    {
        case 3:                             // 内两路都在线上：居中直行
            turn_diff = 0;
            break;
        case 2:                             // 仅DH2(右内)见线 → 车偏左 → 右微调
            turn_diff = -TurnMinAngle;
            break;
        case 1:                             // 仅DH3(左内)见线 → 车偏右 → 左微调
            turn_diff = TurnMinAngle;
            break;
        default:                            // 0: 两内都离线（丢线/弯内/特殊段）→ 沿用上次转向
            turn_diff = last_turn;
            break;
    }
    if(inner != 0) last_turn = turn_diff;   // 只保存有效巡线转向

    // 转向速度越大，基础速度越低
    if(fabs(turn_diff)<ForwardLimit)
        base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
    else base_speed_mm = 0;
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

// Function: IRDM_Line_Seen - 判断是否检测到线（绕障后找线用）
u8 IRDM_Line_Seen(void)   // 1=检测到线，0=丢线/终点
{
	int s = (DH1<<3)|(DH2<<2)|(DH3<<1)|DH4;
	return (s != STATE_LOST && s != STATE_T);   // STATE_END已改名STATE_T(=0全白)；全黑15也不认作“线”
}

// ===== 特殊状态按序匹配状态机（10ms调用一次，仅巡线模式且非绕障时由 control.c 调用） =====
void Special_Seq_Reset(void)
{
	seq_step = 0; seq_cool = 0; seq_hold = 0; seq_hold_ticks = 0; seq_hold_turn = 0;
}

void Special_Seq_Step(void)
{
	static u8 seq_last = 0xFF;              // 上次读取状态（做“新出现”边沿检测）
	u8 s = (DH1<<3)|(DH2<<2)|(DH3<<1)|DH4;

	// 1) 强制转向段：turn_diff/base_speed 压成指令值，直到下一步命中/终点
	if(seq_hold)
	{
		if(++seq_hold_ticks > SEQ_HOLD_MAX_TICKS)   // 兜底：超时退出强制转向，交还内两路巡线
		{ seq_hold = 0; seq_hold_turn = 0; seq_hold_ticks = 0; }
		else
		{
			turn_diff = seq_hold_turn;
			if(fabs(seq_hold_turn)<ForwardLimit)
				base_speed_mm = BaseSpeed - (BaseSpeed*(fabs(seq_hold_turn)/ForwardLimit));
			else base_speed_mm = 0;
			Move_X = base_speed_mm;         // 同步速度环给定，转弯减速
		}
	}

	// 2) 命中后屏蔽期：忽略新状态（强制转向保持）
	if(seq_cool) { seq_cool--; seq_last = s; return; }

	// 3) 边沿：状态需“新出现”才算一次事件，防同一点重复触发/冷却后粘连
	if(s == seq_last) return;
	seq_last = s;

	// 4) 按序匹配当前期待步；乱序/未知状态一律忽略（不触发、不重置）
	if(s != sensor_state_table[seq_step].state) return;

	if(sensor_state_table[seq_step].stop)   // 终点步：停车切普通模式
	{
		Mode = Normal_Mode;
		seq_step = 0; seq_cool = 0; seq_hold = 0; seq_hold_ticks = 0; seq_hold_turn = 0;
	}
	else
	{
		if(sensor_state_table[seq_step].turn != 0)
		{ seq_hold = 1; seq_hold_turn = sensor_state_table[seq_step].turn; seq_hold_ticks = 0; }
		else
		{ seq_hold = 0; seq_hold_turn = 0; seq_hold_ticks = 0; }
		if(++seq_step >= SEQ_STEP_NUM) seq_step = 0;   // 兜底回绕（正常流程应停在终点步）
		seq_cool = SEQ_COOL_TICKS;                    // 命中后屏蔽 N×10ms
	}
}













