#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"

#define KEY_EVENT_NONE         0
#define KEY_EVENT_CLICK        1
#define KEY_EVENT_LONG_PRESS   2
#define KEY_EVENT_DOUBLE_CLICK 3

#define KEY_ON  1   /* PA0 按下为高电平 */

void KEY_Init(void);
int  KEY_Scan(void);   /* 非阻塞，每次调用推进状态机，返回事件码(一次性) */

#endif
