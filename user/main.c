/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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
#include "XYZ_MOTION_task.h"
#include "GRIPPER_task.h"
#include "CATCH_task.h"
#include "RescueHook_task.h"
#include "referee_usart_task.h"

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
       Scheduler_Run();			//运行任务调度器，所有系统功能，除了中断服务函数，都在任务调度器内完成 ;
    }
}

//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //中断组 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //初始化滴答时钟 1ms SysTick中断  中断服务函数位于delay.c
    delay_init(configTICK_RATE_HZ);
    //流水灯，红绿灯初始化
    led_configuration();
    //24V 输出控制口 初始化
    power_ctrl_configuration();
	  //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //17mm和42mm舱门PWM初始化 TIM8 CH3 CH4
    AMMO_OUT_PWM_configuration();
		//救援夹爪PWM初始化 TIM5 CH3:B CH4:A
		HOOK_OUT_PWM_configuration();//A板的PWM口A:PI0 B:PH12
    //激光IO初始化
		laser_configuration();
		//蜂鸣器初始化 TIM12
		buzzer_init(350, 90);
    //CAN接口初始化 1M波特率
		CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
		//底盘初始化
		chassis_Setup();
		//夹爪任务
		CATCH_Setup();
		//旋转夹爪任务
		GRIPPER_Setup();
		//3轴滑台运动
		XYZ_MOTION_Setup();
    //遥控器初始化
    remote_control_init();
		//USART6 + DMA  裁判系统读取	
		Referee_Task_Init();
		
		//初始化完成后蜂鸣器响一声代表初始化成功
		BSPInit_CompleteBeep();
		buzzer_init(30000, 90);
		/* 目前用不着的任务*/
    //TIM6_Init(60000, 90);
		//CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    //flash读取函数，把校准值放回对应参数
    //cali_param_init();
		//Calibration_Init();
		//stm32 板载温度传感器初始化
    //temperature_ADC_init();
}
