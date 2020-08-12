/* 气缸控制的BSP和APP部分 原理：GPIO高低电平控制电磁阀*/
#include "cylinder_gpio.h"

static void CATCH_GPIO_init(void);
static void Rescue_GPIO_init(void);
static void X_Motion_GPIO_Init(void);

void Cylinder_GPIO_Init(void)
{
		X_Motion_GPIO_Init();
		Rescue_GPIO_init();
		CATCH_GPIO_init();
		
}

void GPIO_CMD_Cylinder(Cylinder_num_e Cylinder_num, Cylinder_mode_e mode)
{
	switch((u8)Cylinder_num)
	{
		case X_Motion_Cylinder:
		{
			if(mode == CLOSE) GPIO_SetBits(X_Motion_Cylinder_PORT, X_Motion_Cylinder_PIN);
			else if(mode == OPEN) GPIO_ResetBits(X_Motion_Cylinder_PORT, X_Motion_Cylinder_PIN);
		}
		case Rescue_Cylinder:
		{
			if(mode == CLOSE) GPIO_SetBits(Rescue_Cylinder_PORT, Rescue_Cylinder_PIN);
			else if(mode == OPEN) GPIO_ResetBits(Rescue_Cylinder_PORT, Rescue_Cylinder_PIN);
		}		
		case Catch_Cylinder:
		{
			if(mode == CLOSE) GPIO_SetBits(Catch_Cylinder_PORT, Catch_Cylinder_PIN);
			else if(mode == OPEN) GPIO_ResetBits(Catch_Cylinder_PORT, Catch_Cylinder_PIN);
		}		

	}
	
}

static void X_Motion_GPIO_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(X_Motion_Cylinder_RCC, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = X_Motion_Cylinder_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(X_Motion_Cylinder_PORT, &GPIO_InitStructure);
		
		GPIO_ResetBits(X_Motion_Cylinder_PORT, X_Motion_Cylinder_PIN);

}

static void Rescue_GPIO_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(Rescue_Cylinder_RCC, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = Rescue_Cylinder_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
		
		GPIO_ResetBits(Rescue_Cylinder_PORT, Rescue_Cylinder_PIN);

}

static void CATCH_GPIO_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(Catch_Cylinder_RCC, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = Catch_Cylinder_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
		
		GPIO_ResetBits(Catch_Cylinder_PORT, Catch_Cylinder_PIN);
}


