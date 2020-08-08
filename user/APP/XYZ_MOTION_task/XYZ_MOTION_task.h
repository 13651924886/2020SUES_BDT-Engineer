
#ifndef XYZ_MOTION_TASK_H
#define XYZ_MOTION_TASK_H

#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "user_lib.h"
#include "stm32f4xx.h"

/*********�궨�� ��д***********/
#define Y_MOTION_CONTROL_TIME 0.001
#define Y_MOTION_CONTROL_FREQUENCE 1000.0f


#define Y_MOTION_RC_SEN	-0.006f
#define Y_MOTION_CONTROL_CHANNEL 2
#define Y_MOTION_RC_DEADLINE 10	//ң����ͨ��ֵ����

#define Y_MOTION_MAX_SPEED 1000.0f

#define SychronicWheelRadius 0.025 //0.025m 25mm

#define Y_MOTION_M3505_MOTOR_SPEED_PID_KP 1000.0f
#define Y_MOTION_M3505_MOTOR_SPEED_PID_KI 0.0f
#define Y_MOTION_M3505_MOTOR_SPEED_PID_KD 0.0f
#define Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_OUT 1000.0f
#define Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define Y_MOTION_M3505_MOTOR_POSITION_PID_KP 10.0f
#define Y_MOTION_M3505_MOTOR_POSITION_PID_KI 0.0f
#define Y_MOTION_M3505_MOTOR_POSITION_PID_KD 0.5f
#define Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_OUT 6.0f
#define Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_IOUT 0.2f

#define MOTION_ACCEL_Y_NUM 0.1666666667f
/********************/

/*********�ṹ�������� ��д*****/
typedef enum
{
	Y_MOTION_ENGAGE,
	Y_MOTION_STOP
	
} Y_MOTION_mode_e;

typedef struct
{
  const motor_measure_t *Y_MOTION_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Y_MOTION_Motor_t;

typedef struct
{
  const RC_ctrl_t *Y_MOTION_System_RC;              
  Y_MOTION_mode_e Y_MOTION_mode;              
  Y_MOTION_mode_e last_Y_MOTION_mode;         
  Y_MOTION_Motor_t Y_MOTION_Left_motor;
  Y_MOTION_Motor_t Y_MOTION_Right_motor;   	
  PidTypeDef Y_MOTION_Left_motor_Speed_pid;  
	PidTypeDef Y_MOTION_Left_motor_Position_pid; 
	PidTypeDef Y_MOTION_Right_motor_Speed_pid;
  PidTypeDef Y_MOTION_Right_motor_Position_pid;              

  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vy;                         //��λ m/s  ������ʾ����
  fp32 vy_set;                     //��λ m/s  ������ʾ����
	
	fp32 Position;
	fp32 Position_set;

  fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
  
} XYZ_MOTION_System_t;

/*********�������� *************/
extern XYZ_MOTION_System_t Y_MOTION_move;

extern void XYZ_MOTION_Setup(void);
extern void XYZ_MOTION_task(void);
extern void XYZ_MOTION_rc_filter(fp32 *vy_set, XYZ_MOTION_System_t *Y_MOTION_move);

/********************/





#endif
