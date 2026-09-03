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
// ===== 特殊状态序列：全4路特殊态按序匹配（直角弯/停车） =====
typedef struct {
    u8    state;   // 期待匹配的全4位传感器状态（黑=1）
    int   dir;     // 转向方向：-1=右转 0=不转 +1=左转；幅度固定用 Turn90Angle（运行时可调）
    u8    stop;    // 1=命中后切普通模式停车（终点步）
} SeqStep_t;

SeqStep_t sensor_state_table[] = {                 // 赛道顺序，实测后按需修改
    {STATE_RIGHT_BIG, -1, 0},   // 命中 RIGHT_BIG → 右转 Turn90Angle
    {STATE_T,          1, 0},   // 命中 T(0000)   → 左转 Turn90Angle
    {STATE_LEFT_BIG,   1, 0},   // 命中 LEFT_BIG  → 左转 Turn90Angle
    {STATE_T,         -1, 0},   // 命中 T(0000)   → 右转 Turn90Angle
    {STATE_T,          0, 1},   // 命中 T(0000)   → 终点停车(切普通模式)
};
#define SEQ_STEP_NUM   (sizeof(sensor_state_table)/sizeof(SeqStep_t))
#define SEQ_TURN_TICKS 50           // 命中后原地转向时长：50×10ms=500ms（按需标定）

static u8    seq_step = 0;      // 当前期待步索引
static u8    seq_turn = 0;      // 1=原地转向窗口（转向期间忽略其他状态）
static u16   seq_turn_cnt = 0;  // 原地转向剩余拍数
static float seq_turn_val = 0;  // 原地转向差速值
float base_speed_mm = 0;// 基础速度（mm/s）
float turn_diff = 0;    // 转向差速

// ===== 巡线功能函数（四路全状态巡线，与旧版一致；特殊大片态由特殊状态机按序处理） =====
void IRDM_line_inspection(void)
{
    static int last_state = 0;// 记录上一次的状态

    // 读取传感器状态：4个传感器组合值
    int sensor_state = (DH1 << 3) | (DH2 << 2) | (DH3 << 1) | DH4;
        // ===== 状态判断：设置转向差速 =====
    switch (sensor_state)
    {
       case STATE_T:// T字路口/全白：不自行停车（停车由特殊状态机终点步负责）
			turn_diff = 0;
			//Mode = Normal_Mode;		//（去注释会让首次见0000即停车，绕过特殊序列判定）
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
            turn_diff = -TurnMinAngle;
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
        // ===== Output: base_speed_mm(line speed) & turn_diff(turn differential) =====
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
	seq_step = 0; seq_turn = 0; seq_turn_cnt = 0; seq_turn_val = 0;
}

void Special_Seq_Step(void)
{
	static u8 seq_last = 0xFF;              // 上次读取状态（做“新出现”边沿检测）
	u8 s = (DH1<<3)|(DH2<<2)|(DH3<<1)|DH4;

	// 1) 原地转向窗口：前向给定压 0（原地转，防速度积分空转猛冲），
	//    持续 SEQ_TURN_TICKS 后自动结束，下一拍起交还内两路巡线
	if(seq_turn)
	{
		turn_diff = seq_turn_val;           // 强制差速转向（左+右-）
		Move_X = 0;                         // 停车给定：只转不走
		if(--seq_turn_cnt == 0) seq_turn = 0;   // 到时结束，交还巡线
		return;                             // 转向期间忽略其他状态
	}

	// 2) 边沿：状态需“新出现”才算一次事件，防同一点重复触发
	if(s == seq_last) return;
	seq_last = s;

	// 3) 按序匹配当前期待步；乱序/未知状态一律忽略（不触发、不重置）
	if(s != sensor_state_table[seq_step].state) return;

	if(sensor_state_table[seq_step].stop)   // 终点步：停车切普通模式
	{
		Mode = Normal_Mode;
		seq_step = 0; seq_turn = 0; seq_turn_cnt = 0; seq_turn_val = 0;
	}
	else
	{
		int d = sensor_state_table[seq_step].dir;    // 转向方向；幅度=Turn90Angle（运行时可调）
		if(d != 0)                                   // 非终点且要转向：进入原地转向窗口
		{ seq_turn = 1; seq_turn_cnt = SEQ_TURN_TICKS; seq_turn_val = d * Turn90Angle; }
		else                                         // 无转向动作：直接等下一步
		{ seq_turn = 0; seq_turn_val = 0; }
		if(++seq_step >= SEQ_STEP_NUM) seq_step = 0;   // 兜底回绕（正常流程应停在终点步）
	}
}













