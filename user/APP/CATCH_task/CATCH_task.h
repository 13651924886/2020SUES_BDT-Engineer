#ifndef CATCH_TASK_H
#define CATCH_TASK_H
#include "stm32f4xx.h"

#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "user_lib.h"

typedef enum
{
	CATCH_CLOSE,
	CATCH_OPEN
	
} CATCH_mode_e;

//等加了限位开关再拓展这部分程序 有反馈元件检测夹爪状态
//typedef enum
//{
//	VALVE_HIGH,
//	VALVE_LOW
//	
//} Valve_mode_e;


typedef struct
{
  const RC_ctrl_t *CATCH_System_RC; 
	
  CATCH_mode_e CATCH_Status;              
  CATCH_mode_e last_CATCH_Status;
	
 // CATCH_Mag_Valve_t Mag_Valve;
	
} CATCH_System_t;

extern void CATCH_Setup(void);
extern void CATCH_task(void);

#endif