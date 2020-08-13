#ifndef CATCH_TASK_H
#define CATCH_TASK_H
/*
		抓
*/
#include "stm32f4xx.h"

#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "user_lib.h"
#include "cylinder_gpio.h"

//鼠标长按判断
#define PRESS_LONG_TIME 400
typedef enum
{
	CATCH_STOP,
	CATCH_ENGAGE
	
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
	
	bool_t press_l;
  bool_t press_r;
	
  bool_t last_press_l;
  bool_t last_press_r;
	
  uint16_t press_l_time;
  uint16_t press_r_time;
	
  CATCH_mode_e CATCH_Status;              
  CATCH_mode_e last_CATCH_Status;

	bool_t CATCH_Cylinder_GPIO;
 // CATCH_Mag_Valve_t Mag_Valve;
	void (*ControlFun)(Cylinder_num_e Cylinder_num, Cylinder_mode_e mode);
} CATCH_System_t;


extern void CATCH_Setup(void);
extern void CATCH_task(void);

#endif

