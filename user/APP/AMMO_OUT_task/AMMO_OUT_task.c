#include "AMMO_OUT_task.h"

#include "stm32f4xx.h"

void AMMO_OUT_PWM_configuration(void) //
{

    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE); 	//TIM8时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE); //GPIOI时钟 

////	PI5初始化
//		GPIO_PinAFConfig(GPIOI,GPIO_PinSource5,GPIO_AF_TIM8);	//PI5复用为TIM8外设输出
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                             
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                            
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOI,&GPIO_InitStructure);
//	
////	PI6初始化	
//		GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8);                     
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                             
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                           
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOI,&GPIO_InitStructure);

//	PI7初始化
		GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8);                     
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                              
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                            
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOI,&GPIO_InitStructure);

//	PI2初始化
		GPIO_PinAFConfig(GPIOI,GPIO_PinSource2,GPIO_AF_TIM8);                     
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                              
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                            
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOI,&GPIO_InitStructure);

//    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, ENABLE);
//    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, DISABLE);
//		
/* ----------------   PWM信号 周期和占空比的计算--------------- */
// ARR ：自动重装载寄存器的值
// CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
// PWM 信号的周期 T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// 占空比P=CCR/(ARR+1)
// 若Period = 19,则+1得20，计一次数的周期为1/CLK_CNT = 71+1/72000000 = 0.000001s,则PWM周期为20x0.000001s=0.00002s=20us=50000HZ=50KHZ
//180000 19

//	TIM_TimeBase初始化
    TIM_TimeBaseInitStructure.TIM_Period = 3000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 180 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);

//	TIM_OC初始化
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = 1930;


//		TIM_OC1Init(TIM8, &TIM_OCInitStructure);
//		TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);                         //?????????
//		
//		TIM_OC2Init(TIM8, &TIM_OCInitStructure);
//		TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);                         //?????????
		
		TIM_OC3Init(TIM8, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);                         //?????????
		
		TIM_OC4Init(TIM8, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);                         //?????????
	
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);

    AMMO17_off();
		AMMO42_off();

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOF, &GPIO_InitStructure);
//    GPIO_SetBits(GPIOF, GPIO_Pin_10);
}
void AMMO_42mm_OUT_task(void)
{
	if (switch_is_up(rc_ctrl.rc.s[1]))
		{
			AMMO42_out();		
		}
	if (switch_is_mid(rc_ctrl.rc.s[1]))
		{
			AMMO42_off();		
		}
	if (switch_is_down(rc_ctrl.rc.s[1]))
	{
			AMMO42_off();		
	}
}
void AMMO_17mm_OUT_task(void)
{
		if (switch_is_up(rc_ctrl.rc.s[1]))
		{
			AMMO17_out();		
		}
	if (switch_is_mid(rc_ctrl.rc.s[1]))
		{
			AMMO17_off();		
		}
	if (switch_is_down(rc_ctrl.rc.s[1]))
	{
			AMMO17_off();		
	}
}


void AMMO42_out(void)
{
    TIM_SetCompare3(TIM8, AMMO42_OUT);
}
void AMMO42_off(void)
{
    TIM_SetCompare3(TIM8, AMMO42_OFF);
}
void AMMO17_out(void)
{
    TIM_SetCompare4(TIM8, AMMO17_OUT);
}
void AMMO17_off(void)
{
		TIM_SetCompare4(TIM8, AMMO17_OFF);
}
