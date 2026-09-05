#ifndef __TRACKMODULE_H
#define __TRACKMODULE_H
#include "sys.h"

// 红外传感器引脚（从左往右：PC8、PC4、PC9、PB8；DH4=最左，DH1=最右，与驱动LEFT/RIGHT状态一致）
#define DH1 PCin(8)
#define DH2 PCin(4)
#define DH3 PCin(9)
#define DH4 PBin(8)

// 红外传感器引脚配置宏（与 DH1~DH4 对应，改硬件接线时只需改这里）
#define TRACK_GPIO_CLK2     RCC_APB2Periph_GPIOC                                   // 引脚时钟1
#define TRACK_GPIO_PORT2    GPIOC                                                  // 端口1
#define TRACK_GPIO_PIN2     (GPIO_Pin_8 | GPIO_Pin_4 | GPIO_Pin_9)                 // PC8/PC4/PC9
#define TRACK_GPIO_CLK1     RCC_APB2Periph_GPIOB                                   // 引脚时钟2
#define TRACK_GPIO_PORT1    GPIOB                                                  // 端口2
#define TRACK_GPIO_PIN1     GPIO_Pin_8                                             // PB8

extern float Turn90Angle ;   // 直角弯转向角度
extern float TurnMaxAngle;   // 大弯道转向角度
extern float TurnMidAngle;   // 中等转向角度（丢线时使用）
extern float TurnMinAngle;   // 微调转向角度
extern float BaseSpeed;
extern float ForwardLimit;
// 速度参数（单位：m/s）
extern float base_speed_mm ;        // 基础速度（mm/s）
extern float turn_diff ;            // 转向差速（左+右-，单位：mm/s）

void TrackModule_Init(void);
void IRDM_line_inspection(void);
u8 IRDM_Line_Seen(void);   // 1=检测到线（绕障后找线用）
u8 EightTrack_IsIdle(void); // 1=特殊路况状态机空闲（避障触发互斥用）
void EightTrack_Reset(void); // 重置8字状态机（避障抢占/第一圈终点重置时调用）

typedef enum {
    EIGHT_TRACK_IDLE = 0,
    EIGHT_TRACK_FIRST_TURN_RIGHT,
    EIGHT_TRACK_FIRST_WAIT_END,
    EIGHT_TRACK_FIRST_TURN_LEFT,
    EIGHT_TRACK_WAIT_LEFT_ANGLE,
    EIGHT_TRACK_SECOND_TURN_LEFT,
    EIGHT_TRACK_SECOND_WAIT_END,
    EIGHT_TRACK_SECOND_TURN_RIGHT
} EightTrackState_t;

extern EightTrackState_t eight_track_state; // 当前状态机阶段（OLED诊断显示用）
#endif

