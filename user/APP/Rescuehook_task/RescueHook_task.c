#include "RescueHook_task.h"


static void RescueHook_ReachOut(void);
static void RescueHook_Reset(void);


void HOOK_OUT_PWM_configuration()
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	//ʹ��PORTHʱ��	
	
//PH12	 B
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5); //GPIOH12����Ϊ��ʱ��5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           //GPIOH12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOH,&GPIO_InitStructure);              //��ʼ��GPIOH
//PI0		 A
	GPIO_PinAFConfig(GPIOI,GPIO_PinSource0,GPIO_AF_TIM5); //GPIOH12����Ϊ��ʱ��5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;           //GPIOH12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOI,&GPIO_InitStructure);              //��ʼ��GPIOH 
	
//TIM5_CH3	B
	TIM_TimeBaseStructure.TIM_Prescaler=180 - 1;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=3000 - 1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//��ʼ����ʱ��5
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_Pulse = 1930;
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 4OC2
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM15��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM5
	
//TIM5_CH4	A	  
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_Pulse = 1930;
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 4OC2
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM15��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM5
	
	RescueHook_Reset();
}

void Rescue_task(void)
{
	if (switch_is_up(rc_ctrl.rc.s[1]))
		{
			RescueHook_Reset();		
		}
	if (switch_is_mid(rc_ctrl.rc.s[1]))
		{
			RescueHook_ReachOut();		
		}
	if (switch_is_down(rc_ctrl.rc.s[1]))
	{
			RescueHook_Reset();		
	}
}

static void RescueHook_ReachOut(void)
{
	 TIM_SetCompare3(TIM5, 2980);  //B
	 TIM_SetCompare4(TIM5, 300);	 //A

}

static void RescueHook_Reset(void)
{
	 TIM_SetCompare3(TIM5, 450);
	 TIM_SetCompare4(TIM5, 1050);
}
