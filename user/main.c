/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��h�ļ��������ȫ�ֺ궨���Լ�
  *             typedef һЩ������������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"

#include "exit_init.h"
#include "chassis_task.h"
#include "AMMO_OUT_task.h"
#include "Y_MOTION_task.h"
#include "GRIPPER_task.h"
#include "CATCH_task.h"
#include "RescueHook_task.h"

#include "remote_control.h"
#include "start_task.h"

#define configTICK_RATE_HZ 1000

void BSP_init(void);

int main(void)
{
		BSP_init();
    delay_ms(100);
		Scheduler_Setup();
    while (1)
    {
       Scheduler_Run();			//�������������������ϵͳ���ܣ������жϷ������������������������� ;
    }
}

//�ĸ�24v ��� ���ο��� ��� 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //�ж��� 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //��ʼ���δ�ʱ�� 1ms SysTick�ж�  �жϷ�����λ��delay.c
    delay_init(configTICK_RATE_HZ);
    //��ˮ�ƣ����̵Ƴ�ʼ��
    led_configuration();
    //stm32 �����¶ȴ�������ʼ��
    //temperature_ADC_init();
    //24������ƿ� ��ʼ��
    power_ctrl_configuration();
    //17mm��42mm����PWM��ʼ�� TIM8 CH3 CH4
    AMMO_OUT_PWM_configuration();
		//��Ԯ��צPWM��ʼ�� TIM5 CH3:B CH4:A
		HOOK_OUT_PWM_configuration();//A���PWM��A:PI0 B:PH12
    //����IO��ʼ��
		laser_configuration();
		//MPU6500��IST8310��ʼ��   MPU6500ΪSPI5+DMA  IST8310Ϊģ��IIC
		//��������ʼ�� TIM12
		buzzer_init(350, 90);
		 //��ʱ��6 ��ʼ��
    //TIM6_Init(60000, 90);
    //CAN�ӿڳ�ʼ��
		CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
		//���̳�ʼ��
		chassis_Setup();		
		GRIPPER_Setup();
		Y_MOTION_Setup();
		CATCH_Setup();
    //24v ��� �����ϵ�
//    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
//    {
//        power_ctrl_on(i);
//        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
//    }
    //ң������ʼ��
    remote_control_init();
    //flash��ȡ��������У׼ֵ�Żض�Ӧ����
    //cali_param_init();
		//Calibration_Init();
		BSPInit_CompleteBeep();
		buzzer_init(30000, 90);
		//CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
		delay_ms(2000);
		//AMMO17_out();
//		AMMO42_out();
}
