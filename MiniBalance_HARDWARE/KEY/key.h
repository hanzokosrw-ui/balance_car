#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#define KEY PAin(0)
#define KEY_ON	1
#define KEY_OFF	0
//用户按键返回值状态
#define No_Action 					0
#define Click 						1
#define Long_Press 					2
#define Double_Click				3
#define KEY2_STATE  		 PCin(13)

void KEY_Init(void);          //按键初始化
uint8_t User_Key_Scan(void);
void Mode_Choose(void);
#endif  
