/* 气缸控制的BSP部分 原理：GPIO高低电平控制电磁阀*/
#ifndef _CYLINDER_GPIO_
#define _CYLINDER_GPIO_

#include "stm32f4xx.h"

// 1号气缸
#define X_Motion_Cylinder_PORT  GPIOA
#define X_Motion_Cylinder_PIN 	GPIO_Pin_2
#define X_Motion_Cylinder_RCC 	RCC_AHB1Periph_GPIOF


// 2号气缸
#define Rescue_Cylinder_PORT 		GPIOA
#define Rescue_Cylinder_PIN	 		GPIO_Pin_2
#define Rescue_Cylinder_RCC			RCC_AHB1Periph_GPIOF

// 3号气缸
#define Catch_Cylinder_PORT 		GPIOA
#define Catch_Cylinder_PIN	 		GPIO_Pin_2
#define Catch_Cylinder_RCC			RCC_AHB1Periph_GPIOF


typedef enum
{
	X_Motion_Cylinder = 0,
	Rescue_Cylinder,
	Catch_Cylinder,
	
}Cylinder_num_e;

typedef enum
{
	CLOSE,
	OPEN,
	
}Cylinder_mode_e;

//等加了限位开关再拓展这部分程序 有反馈元件检测夹爪状态
//typedef enum
//{
//	VALVE_HIGH,
//	VALVE_LOW
//	
//} Valve_mode_e;

extern void Cylinder_GPIO_Init(void);

extern void GPIO_CMD_Cylinder(Cylinder_num_e Cylinder_num, Cylinder_mode_e mode);

#endif

