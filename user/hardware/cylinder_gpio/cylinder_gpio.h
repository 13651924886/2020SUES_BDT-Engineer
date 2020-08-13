/* ���׿��Ƶ�BSP���� ԭ��GPIO�ߵ͵�ƽ���Ƶ�ŷ�*/
#ifndef _CYLINDER_GPIO_
#define _CYLINDER_GPIO_

#include "stm32f4xx.h"

// 1������
#define X_Motion_Cylinder_PORT  GPIOA
#define X_Motion_Cylinder_PIN 	GPIO_Pin_2
#define X_Motion_Cylinder_RCC 	RCC_AHB1Periph_GPIOF


// 2������
#define Rescue_Cylinder_PORT 		GPIOA
#define Rescue_Cylinder_PIN	 		GPIO_Pin_2
#define Rescue_Cylinder_RCC			RCC_AHB1Periph_GPIOF

// 3������
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

//�ȼ�����λ��������չ�ⲿ�ֳ��� �з���Ԫ������צ״̬
//typedef enum
//{
//	VALVE_HIGH,
//	VALVE_LOW
//	
//} Valve_mode_e;

extern void Cylinder_GPIO_Init(void);

extern void GPIO_CMD_Cylinder(Cylinder_num_e Cylinder_num, Cylinder_mode_e mode);

#endif

