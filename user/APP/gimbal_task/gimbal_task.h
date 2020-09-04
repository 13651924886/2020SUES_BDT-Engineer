#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"
#include "arm_math.h"
#include "user_lib.h"

/*** 需要调的参数 ***/
	/*** 云台部分 ***/
//云台relative angle offset 根据实际装配来设置偏差
#define GIMBAL_YAW_RELATIVE_ANGLE_OFFSET 4358		
#define GIMBAL_PITCH_RELATIVE_ANGLE_OFFSET 3680  

//云台relative angle最大值 根据实际装配来设置最大值
#define GIMBAL_YAW_MAX_RELATIVE_ANGLE 0.2  //rad
#define GIMBAL_PITCH_MAX_RELATIVE_ANGLE 0.45
//云台relative angle最小值 根据实际装配来设置最大值
#define GIMBAL_YAW_MIN_RELATIVE_ANGLE -0.2 //rad
#define GIMBAL_PITCH_MIN_RELATIVE_ANGLE -0.25

//云台电机正反装
#define PITCH_TURN 0
#define YAW_TURN 0

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 0.0f   				// P
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f						// I
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f   				// D
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 100.0f 		//角度环输出限幅
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f	 		//角度环积分限幅
//yaw 速度环
#define YAW_SPEED_PID_KP 20000.0f									// P
#define YAW_SPEED_PID_KI 40.0f										// I
#define YAW_SPEED_PID_KD 2000.0f                  // D
#define YAW_SPEED_PID_MAX_OUT 30000.0f						//速度环输出限幅
#define YAW_SPEED_PID_MAX_IOUT 5000.0f						//速度环积分限幅

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 0.0f					// P
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f        // I
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f         // D

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f   //角度环输出限幅
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f		//角度环积分限幅

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 15000.0f								// P
#define PITCH_SPEED_PID_KI 30.0f									// I
#define PITCH_SPEED_PID_KD 0.0f										// D
#define PITCH_SPEED_PID_MAX_OUT 30000.0f					//速度环输出限幅
#define PITCH_SPEED_PID_MAX_IOUT 5000.0f					//速度环积分限幅

	/**** 灵敏度 ****/
//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN -0.000005f
#define Pitch_RC_SEN 0.000006f //0.005
//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_Sen 0.000025f
#define Pitch_Mouse_Sen -0.000015f
//云台编码器控制时候使用的比例
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f

/****************************************************/

/**** 软硬件相关的固定参数 ****/
	//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0
#define Swing 1
	//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
	//云台控制周期（ms）
#define GIMBAL_CONTROL_TIME 1
	//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10
	//2006电机属性
#define Motor_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f  // 对于2006 传动比为36 那么该系数 =（10度/8191） *（PI/180）
#define FULL_COUNT 18

typedef enum
{
    GIMBAL_MOTOR_STOP = 0, //无力模式
    GIMBAL_MOTOR_ENCONDE, //编码器模式
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
	
    Gimbal_PID_t gimbal_motor_relative_angle_pid;
	  PidTypeDef gimbal_motor_speed_pid;
	
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
	
		uint16_t offset_ecd;
		
		int8_t ecd_count;
	
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad

    fp32 motor_speed;
		fp32 motor_speed_set;
	
    fp32 current_set;
    int16_t given_current;

} Gimbal_Motor_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;    //遥控器指针
    Gimbal_Motor_t gimbal_yaw_motor;    //YAW电机结构体
    Gimbal_Motor_t gimbal_pitch_motor;	//PITCH电机结构体
} Gimbal_System_t;

void Gimbal_Setup(void);
void Gimbal_Task(void);





#endif 
