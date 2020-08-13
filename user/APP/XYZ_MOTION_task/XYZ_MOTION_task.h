
#ifndef XYZ_MOTION_TASK_H
#define XYZ_MOTION_TASK_H

#include "stm32f4xx.h"
#include "main.h"

#include "cylinder_gpio.h"

#include "pid.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "user_lib.h"


/*********宏定义 待写***********/
#define Y_MOTION_CONTROL_TIME 0.001f				//seconds
#define Y_MOTION_CONTROL_FREQUENCE 1000.0f	//HZ


#define Y_MOTION_RC_SEN	-0.006f
#define Y_MOTION_CONTROL_CHANNEL 2
#define Y_MOTION_RC_DEADLINE 10	//遥控器通道值死区

#define Y_MOTION_MAX_POSITION	980000.0f
#define Y_MOTION_MIN_POSITION 0.0f

#define Z_MOTION_MAX_POSITION	490000.0f
#define Z_MOTION_MIN_POSITION 0.0f

#define Y_MID_POSITION 		0.0f
#define Y_POSITION_ADD		490000.0f

#define Z_INIT_POSITION		0.0f
#define Z_POSITION_ADD		490000.0f


/* Y Motion PID参数*/
#define Y_MOTION_M3505_MOTOR_SPEED_PID_KP 100.0f
#define Y_MOTION_M3505_MOTOR_SPEED_PID_KI 0.0f
#define Y_MOTION_M3505_MOTOR_SPEED_PID_KD 0.0f
#define Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_OUT 20000.0f
#define Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define Y_MOTION_M3505_MOTOR_POSITION_PID_KP 5.0f
#define Y_MOTION_M3505_MOTOR_POSITION_PID_KI 0.0f
#define Y_MOTION_M3505_MOTOR_POSITION_PID_KD 0.0f
#define Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_OUT 100.0f
#define Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_IOUT 0.2f

/* Z Motion PID参数*/
#define Z_MOTION_M3505_MOTOR1_SPEED_PID_KP 100.0f
#define Z_MOTION_M3505_MOTOR1_SPEED_PID_KI 0.0f
#define Z_MOTION_M3505_MOTOR1_SPEED_PID_KD 0.0f
#define Z_MOTION_M3505_MOTOR1_SPEED_PID_MAX_OUT 20000.0f
#define Z_MOTION_M3505_MOTOR1_SPEED_PID_MAX_IOUT 2000.0f

#define Z_MOTION_M3505_MOTOR1_POSITION_PID_KP 5.0f
#define Z_MOTION_M3505_MOTOR1_POSITION_PID_KI 0.0f
#define Z_MOTION_M3505_MOTOR1_POSITION_PID_KD 0.0f
#define Z_MOTION_M3505_MOTOR1_POSITION_PID_MAX_OUT 100.0f
#define Z_MOTION_M3505_MOTOR1_POSITION_PID_MAX_IOUT 0.2f

#define Z_MOTION_M3505_MOTOR2_SPEED_PID_KP 200.0f
#define Z_MOTION_M3505_MOTOR2_SPEED_PID_KI 0.0f
#define Z_MOTION_M3505_MOTOR2_SPEED_PID_KD 0.0f
#define Z_MOTION_M3505_MOTOR2_SPEED_PID_MAX_OUT 20000.0f
#define Z_MOTION_M3505_MOTOR2_SPEED_PID_MAX_IOUT 2000.0f

#define Z_MOTION_M3505_MOTOR2_POSITION_PID_KP 5.0f
#define Z_MOTION_M3505_MOTOR2_POSITION_PID_KI 0.0f
#define Z_MOTION_M3505_MOTOR2_POSITION_PID_KD 0.0f
#define Z_MOTION_M3505_MOTOR2_POSITION_PID_MAX_OUT 100.0f
#define Z_MOTION_M3505_MOTOR2_POSITION_PID_MAX_IOUT 0.2f

#define MOTION_ACCEL_Y_NUM 0.1666666667f

#define KEYBOARD_Y_MOTION_LEFT_SPEED_Y 1.2f
#define KEYBOARD_Y_MOTION_RIGHT_SPEED_Y 1.2f

/* XYZ控制键位 */
#define Y_MOTION_LEFT_KEY 	KEY_PRESSED_OFFSET_Q
#define Y_MOTION_RIGHT_KEY 	KEY_PRESSED_OFFSET_E

#define X_MOTION_KEY 				KEY_PRESSED_OFFSET_SHIFT

#define Z_MOTION_KEY 				KEY_PRESSED_OFFSET_F


//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
#define M3508_FULL_COUNT 19 //		传动比3591/187 = 19

/*********结构体类声明*****/
typedef enum
{
	STOP,
	HOME,
	ENGAGE,
	
} XYZ_MOTION_mode_e;

typedef struct
{
  const motor_measure_t *XYZ_MOTION_motor_measure;
  fp32 accel;
  fp32 angular_speed;	//RPM
  fp32 speed_set;
	fp32 current_set;
	int32_t round_count;
  int16_t give_current;
} XYZ_MOTION_Motor_t;

typedef struct
{
//	XYZ_MOTION_mode_e  Y_MOTION_mode;
//	XYZ_MOTION_mode_e  last_Y_MOTION_mode; 
	
	XYZ_MOTION_Motor_t Y_MOTION_motor;

	PidTypeDef Y_MOTION_motor_Speed_pid;  
	PidTypeDef Y_MOTION_motor_Position_pid;
	
//	ramp_function_source_t Y_cmd_ramp;
	
  fp32 v;                         //单位 m/s  正负表示方向
	
	fp32 Position;
	fp32 Position_set;
	
	fp32 Position_Max;
	fp32 Position_Min;
	
	fp32 current_set;
  int16_t given_current;
	
}Y_MOTION_System;

typedef struct
{
//	XYZ_MOTION_mode_e  X_MOTION_mode;
//	XYZ_MOTION_mode_e  last_X_MOTION_mode; 
	
	Cylinder_mode_e X_MOTION_Cylinder_mode;
	Cylinder_mode_e last_X_MOTION_Cylinder_mode;

	void (*ControlFun)(Cylinder_num_e Cylinder_num, Cylinder_mode_e mode);
}X_MOTION_System;

typedef struct
{
//	XYZ_MOTION_mode_e  Z_MOTION_mode;
//	XYZ_MOTION_mode_e  last_Z_MOTION_mode; 
	
	XYZ_MOTION_Motor_t Z_MOTION_motor1;
	XYZ_MOTION_Motor_t Z_MOTION_motor2;

	PidTypeDef Z_MOTION_motor1_Speed_pid;  
	PidTypeDef Z_MOTION_motor1_Position_pid; 
	
	PidTypeDef Z_MOTION_motor2_Speed_pid;  
	PidTypeDef Z_MOTION_motor2_Position_pid; 
	
//	ramp_function_source_t Z_cmd_ramp;

	fp32 v;                         //单位 m/s  正负表示方向
  fp32 v_set;                     //单位 m/s  正负表示方向
	
	fp32 Position;
	fp32 Position_set;
	
	fp32 Position_Max;
	fp32 Position_Min;

//  fp32 v_max_speed;  //左右方向最大速度 单位m/s
//  fp32 v_min_speed;  //左右方向最小速度 单位m/s
	
}Z_MOTION_System;

typedef struct
{
  const RC_ctrl_t *XYZ_MOTION_System_RC; 
	XYZ_MOTION_mode_e XYZ_mode;
	XYZ_MOTION_mode_e last_XYZ_mode;
	
	Y_MOTION_System Y_MOTION_System;
	X_MOTION_System X_MOTION_System;
	Z_MOTION_System Z_MOTION_System;
  
} XYZ_MOTION_System_t;

/*********函数声明 *************/
extern XYZ_MOTION_System_t XYZ_MOTION_move;

extern void XYZ_MOTION_Setup(void);
extern void XYZ_MOTION_task(void);
extern void XYZ_MOTION_rc_filter(fp32 *v_set, XYZ_MOTION_System_t *Y_MOTION_move);

/********************/





#endif
