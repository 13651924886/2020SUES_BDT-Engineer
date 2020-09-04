/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       start_task.c/h
  * @brief      �������񣬽�һ������������������Դ�������������ȼ�,
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
#ifndef START_TASK_H
#define START_TASK_H
#include "main.h"

#include "Detect_Task.h"
#include "Chassis_Task.h"
#include "delay.h"
#include "User_Task.h"
#include "buzzer.h"
#include "XYZ_MOTION_task.h"
#include "GRIPPER_task.h"
#include "CATCH_task.h"
#include "CAN_Receive.h"
#include "AMMO_OUT_task.h"
#include "Revive_task.h"
#include "RescueHook_task.h"

#include "referee_usart_task.h"


//͵����ʱ�ӵ� ֮��Ҫд�ɶ���
#include "Remote_control.h"
#include "AMMO_OUT_task.h"
typedef struct
{
void(*task_func)(void);
uint16_t rate_hz;
uint16_t interval_ticks;
uint32_t last_run;
}sched_task_t;

//void startTast(void);
void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif
