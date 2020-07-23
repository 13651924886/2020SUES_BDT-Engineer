#ifndef GIMBALTASK_H
#define GIMBALTASK_H

#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "user_lib.h"

/*********�궨�� ��д***********/
#define GRIPPER_CONTROL_TIME 0.001
#define GRIPPER_CONTROL_FREQUENCE 1000.0f


#define GRIPPER_RC_SEN	-3.5f
#define GRIPPER_CONTROL_CHANNEL 1
#define GRIPPER_RC_DEADLINE 10	//ң����ͨ��ֵ����

#define GRIPPER_MAX_SPEED 1000.0f
#define GRIPPER_MAX_POSITION 100
#define GRIPPER_MIN_POSITION 5

//�������ֵ����Լ���ֵ
#define Half_ecd_range 4096
#define ecd_range 8191

#define GRIPPER_MOTOR1_RELATIVE_ANGLE_OFFSET 6000
#define GRIPPER_MOTOR2_RELATIVE_ANGLE_OFFSET 6000


#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192

#define GRIPPER_M3505_MOTOR_SPEED_PID_KP 15.0f
#define GRIPPER_M3505_MOTOR_SPEED_PID_KI 1.0f
#define GRIPPER_M3505_MOTOR_SPEED_PID_KD 2.0f
#define GRIPPER_M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define GRIPPER_M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define GRIPPER_M3505_MOTOR_POSITION_PID_KP 10.0f
#define GRIPPER_M3505_MOTOR_POSITION_PID_KI 0.0f
#define GRIPPER_M3505_MOTOR_POSITION_PID_KD 0.5f
#define GRIPPER_M3505_MOTOR_POSITION_PID_MAX_OUT 6.0f
#define GRIPPER_M3505_MOTOR_POSITION_PID_MAX_IOUT 0.2f

#define MOTION_ACCEL_Y_NUM 0.1666666667f
/********************/

/*********�ṹ�������� ��д*****/
typedef enum
{
	GRIPPER_ENGAGE,
	GRIPPER_STOP
	
} GRIPPER_mode_e;

typedef struct
{
  const motor_measure_t *gripper_motor_measure;
	uint16_t offset_ecd;		
  fp32 relative_angle;     //rad
	fp32 relative_angle_set; //rad
	fp32 accel;
  fp32 speed;
  fp32 speed_set;
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

  first_order_filter_type_t Gripper_cmd_slow_set_Angular_Velocity;

	fp32 Angular_Velocity;								 //��תת�٣�RPM��
	fp32 Angular_Velocity_set;						 //��תת��������RPM��
	
  uint16_t Position;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  uint16_t Position_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	
  fp32 Angular_Velocity_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 Angular_Velocity_min_speed;  //���ҷ�����С�ٶ� ��λm/s
	
	fp32 Position_Max;
	fp32 Position_Min;
  
} GRIPPER_System_t;

/*********�������� *************/
extern GRIPPER_System_t GRIPPER_move;

extern void GRIPPER_Setup(void);
extern void GRIPPER_task(void);
void GRIPPER_rc_filter(fp32 *vy_set, GRIPPER_System_t *Gripper_move);

#endif
