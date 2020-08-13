#ifndef GIMBALTASK_H
#define GIMBALTASK_H

#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "user_lib.h"
/*********宏定义 待写***********/
#define GRIPPER_CONTROL_TIME 0.001
#define GRIPPER_CONTROL_FREQUENCE 1000.0f


#define GRIPPER_RC_SEN	-3.5f
#define GRIPPER_CONTROL_CHANNEL 1
#define GRIPPER_RC_DEADLINE 10	//遥控器通道值死区

#define GPRIPPER_MOTOR1_INIT_POSITION 0
#define GPRIPPER_MOTOR2_INIT_POSITION 0
#define GRIPPER_MAX_POSITION 100000
#define GRIPPER_MIN_POSITION 0
#define GRIPPER_ADD_POSITION 50000

#define GPRIPPER_KEY KEY_PRESSED_OFFSET_F

//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191


#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192

#define GRIPPER_M3505_MOTOR1_SPEED_PID_KP 15.0f
#define GRIPPER_M3505_MOTOR1_SPEED_PID_KI 1.0f
#define GRIPPER_M3505_MOTOR1_SPEED_PID_KD 2.0f
#define GRIPPER_M3505_MOTOR1_SPEED_PID_MAX_OUT 16000.0f
#define GRIPPER_M3505_MOTOR1_SPEED_PID_MAX_IOUT 2000.0f

#define GRIPPER_M3505_MOTOR1_POSITION_PID_KP 10.0f
#define GRIPPER_M3505_MOTOR1_POSITION_PID_KI 0.0f
#define GRIPPER_M3505_MOTOR1_POSITION_PID_KD 0.5f
#define GRIPPER_M3505_MOTOR1_POSITION_PID_MAX_OUT 6.0f
#define GRIPPER_M3505_MOTOR1_POSITION_PID_MAX_IOUT 0.2f

#define GRIPPER_M3505_MOTOR2_SPEED_PID_KP 15.0f
#define GRIPPER_M3505_MOTOR2_SPEED_PID_KI 1.0f
#define GRIPPER_M3505_MOTOR2_SPEED_PID_KD 2.0f
#define GRIPPER_M3505_MOTOR2_SPEED_PID_MAX_OUT 16000.0f
#define GRIPPER_M3505_MOTOR2_SPEED_PID_MAX_IOUT 2000.0f

#define GRIPPER_M3505_MOTOR2_POSITION_PID_KP 10.0f
#define GRIPPER_M3505_MOTOR2_POSITION_PID_KI 0.0f
#define GRIPPER_M3505_MOTOR2_POSITION_PID_KD 0.5f
#define GRIPPER_M3505_MOTOR2_POSITION_PID_MAX_OUT 6.0f
#define GRIPPER_M3505_MOTOR2_POSITION_PID_MAX_IOUT 0.2f

#define MOTION_ACCEL_Y_NUM 0.1666666667f
/********************/

/*********结构体类声明 待写*****/
typedef enum
{
	GRIPPER_STOP,
	GRIPPER_HOME,
	GRIPPER_ENGAGE,
	
} GRIPPER_mode_e;

typedef struct
{
  const motor_measure_t *gripper_motor_measure;
	
	second_order_filter_type_t second_order_filter;
	
	fp32 accel;
  fp32 speed;
	fp32 speed_set;
	
	int32_t round_count;
	int64_t Position;                        
  int64_t Position_set;

	fp32 current_set;	
  int16_t give_current;
	
} GRIPPER_Motor_t;


typedef struct
{
  const RC_ctrl_t *GRIPPER_System_RC; 
	
  GRIPPER_mode_e GRIPPER_mode;              
  GRIPPER_mode_e last_GRIPPER_mode;
	
  GRIPPER_Motor_t GRIPPER_1_motor;
  GRIPPER_Motor_t GRIPPER_2_motor;
	
  PidTypeDef GRIPPER_1_motor_Angular_Speed_pid;  
	PidTypeDef GRIPPER_1_motor_Position_pid; 
	PidTypeDef GRIPPER_2_motor_Angular_Speed_pid;
  PidTypeDef GRIPPER_2_motor_Position_pid;              
	
	fp32 Position_Max;
	fp32 Position_Min;
  
} GRIPPER_System_t;

/*********函数声明 *************/
extern GRIPPER_System_t GRIPPER_move;

extern void GRIPPER_Setup(void);
extern void GRIPPER_task(void);
void GRIPPER_rc_filter(fp32 *vy_set, GRIPPER_System_t *Gripper_move);

#endif
