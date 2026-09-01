#ifndef __TRACKMODULE_H
#define __TRACKMODULE_H
#include "sys.h"

#define DH4 PAin(3)
#define DH3 PAin(2)
#define DH2 PAin(11)
#define DH1 PAin(8)

// 红外传感器引脚配置宏（与 DH1~DH4 对应，改硬件接线时只需改这里）
#define TRACK_GPIO_CLK      RCC_APB2Periph_GPIOA                                   // 引脚时钟
#define TRACK_GPIO_PORT     GPIOA                                                  // 引脚端口
#define TRACK_GPIO_PIN      (GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_2 | GPIO_Pin_3)   // DH1~DH4

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
#endif

