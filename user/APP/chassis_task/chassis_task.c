/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      完成底盘控制任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 BDT****************************
  */
	
/************************* 头文件 ******************************/
	
#include "chassis_task.h"

#include "rc.h"

#include "arm_math.h"

#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "Remote_Control.h"
#include "delay.h"

#include "chassis_behaviour.h"

/************************* 宏定义 ******************************************************************/

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
		
/*********************** 全局变量定义 ***************************************************************/
		

//底盘运动数据结构体定义
//static chassis_move_t chassis_move;
chassis_move_t chassis_move;


/*********************** 静态函数声明 ***************************************************************/
		
//底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
//底盘状态机选择，通过遥控器的开关
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘状态改变后处理控制量的改变static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//底盘设置根据遥控器控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//底盘PID计算以及运动分解
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);


/*********************** 函数定义 *********************************************************************************/

void chassis_Setup(void)
{
    //底盘初始化
    chassis_init(&chassis_move);
    //判断底盘电机是否都在线
//    while (toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE) || toe_is_error(DBUSTOE))
//    {
//        delay_ms(CHASSIS_CONTROL_TIME_MS);
//    }
}
//主任务
/************************ 主任务 ******************************************************/
void chassis_task(void)
{			
        
        chassis_set_mode(&chassis_move);										//遥控器设置状态
 
        chassis_mode_change_control_transit(&chassis_move); //遥控器状态切换数据保存
 
        chassis_feedback_update(&chassis_move);//更新chassis_move	->	motor_chassis[i].speed & accel 速度和加速度
																							 //		 chassis_move	->	vx & vy & wz									 底盘三轴方向速度 编码器
																							 //		 chassis_move_update->chassis_yaw & pitch & roll 底盘三轴欧拉角   陀螺仪
	
        chassis_set_contorl(&chassis_move); 	 //底盘控制量设置，设置遥控器输入控制量
	
        chassis_control_loop(&chassis_move);	 //底盘控制PID计算

	//这里我把DetectTask的事情全部省略，为了方便程序移植，因此不会检测遥控器是否掉线，会一直有输出即代码104行的CAN_CMD_CHASSIS()
//        if (!(toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE)))
//        {
//            //当遥控器掉线的时候，为relax状态，底盘电机指令为零，为了保证一定发送为零，故而不采用设置give_current的方法
//            if (toe_is_error(DBUSTOE))
//            {
//                CAN1_CMD_CHASSIS(0, 0, 0, 0);
//            }
//            else
//            {
//                CAN1_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
//                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
//            }
//        }  

				CAN1_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);				

}

/*********************** 主任务初始化init ************************************************/
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //底盘角度环pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    //底盘一阶滤波参数
		const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;	//CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW
																													//CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW
																													//CHASSIS_VECTOR_NO_FOLLOW_YAW
/**** 遥控器数据指针获取 ****/																												//CHASSIS_VECTOR_RAW
    
    chassis_move_init->chassis_RC = get_remote_control_point();

/**** 陀螺仪数据指针获取 ****/
    
		//获取陀螺仪姿态角指针&INS_Angle[3]   
//		chassis_move_init->chassis_INS_angle = get_INS_angle_point();

/**** 电机数据指针获取 ****/
    
		//获取云台电机数据指针 pitch轴和yaw轴的
//    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
//    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

/**** 初始化YAW和Pitch的PID参数 ****/		
    
		//初始化PID 运动
    for (i = 0; i < 4; i++)
    {		//获取底盘电机m3508的编码器从CAN总线返回的数据
				//一共4个m3508因此结构体数组有4个成员
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        //初始化4个电机各自的速度环PID，设置为位置式PID并将motor_speed_pid[3]的Kp,Ki,Kd参数输入，最后限幅		
				PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //初始化角度环PID，设置为位置式PID并将chassis_yaw_pid的Kp,Ki,Kd参数输入，最后限幅										   
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
   
		//用一阶滤波代替斜波函数生成
		//初始化后才可以使用一阶低通滤波
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //限幅X方向和Y方向的运动速度【MAX and MIN】
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/************************************	1.chassis_Set_Mode ****************************************************/
//第一层chassis_set_mode(chassis_move_t *chassis_move_mode)  																	[chassis_task.c 			static]

//第二层	chassis_behaviour_mode_set(chassis_move_mode)																				[chassis_behaviour.c global]
//				根据遥控器通道值来设置底盘行为状态机，给chassis_behaviour_mode赋值
//				如下6种模式
//				 CHASSIS_ZERO_FORCE,                  //底盘无力
//  			 CHASSIS_NO_MOVE,                     //底盘保持不动
//  			 CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //正常步兵底盘跟随云台
//  			 CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, //工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度
//  			 CHASSIS_NO_FOLLOW_YAW,               //底盘不跟随角度，角度是开环的，但前后左右是有速度环
//  			 CHASSIS_OPEN                         //遥控器的值乘以比例直接发送到can总线上
//
//				再根据底盘行为状态机chassis_behaviour_mode的值赋值底盘状态机chassis_move->chassis_mode
//				如下3种状态值
// 				 CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,		//底盘随动云台
//   			 CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,		//底盘随动底盘
//  			 CHASSIS_VECTOR_NO_FOLLOW_YAW,				//底盘不随动
//  			 CHASSIS_VECTOR_RAW,									//原始底盘

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	//检测是否输入是否为空指针
	//如果是，则返回，防止错误设置底盘模式
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
}


/**************************** 2.chassis_mode_change_control_transit ****************************/

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	//检测是否输入是否为空指针
	//如果是，则返回，防止错误设置底盘模式
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}


/********************************** 3.chassis_feedback_update ****************************************************/
//更新chassis_move	->	motor_chassis[i].speed & accel 速度和加速度
//		chassis_move	->	vx & vy & wz									 底盘三轴方向速度
//		chassis_move	->	chassis_yaw & pitch & roll 底盘三轴欧拉角
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
	/*********** 更新chassis_move->motor_chassis[i].speed和accel ***********/
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机反馈的实际速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *
																										( chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm);
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * 
																											CHASSIS_CONTROL_FREQUENCE;
    }

	/********** 根据更新的chassis_move->motor_chassis[i].speed来计算Vx，Vy，Wz ******************************/
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed 
																+ chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed
																)* MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed 
																+ chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed
																) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed
																- chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed
																) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

}

//遥控器的数据处理成底盘的vx_set，vy_set
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
		
//死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = KEYBOARD_CHASSIS_SPEED_X;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = -KEYBOARD_CHASSIS_SPEED_X;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = KEYBOARD_CHASSIS_SPEED_Y;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = -KEYBOARD_CHASSIS_SPEED_Y;
    }

//一阶低通滤波代替斜波作为底盘速度输入
		// In:vx_set_channel   Out:chassis_move->chassis_cmd_slow_set_vx
		// In:vy_set_channel   Out:chassis_move->chassis_cmd_slow_set_vy
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

//停止信号，不需要缓慢加速，直接减速到零
		//拨杆回中立即速度为0
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}


/**************************** 4.chassis_set_contorl **********************************************************************************/
//底盘控制量设置
//第一层chassis_set_contorl(chassis_move_t *chassis_move_control)																[chassis_task.c 	global]

//第二层	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control)		[chasssis_behaviour.c static]
//				根据底盘行为状态机chassis_behaviour_mode来相应计算vx_set，vy_set，angle_set	[local variables] 
//				
//第二层 	根据底盘控制状态机chassis_move->chassis_mode计算相应的chassis_move->vx_set|vy_set|wz_set期望
//

/****************  第一层 底盘控制量设置 ************************************************************************************/
//设置遥控器输入控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    //设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
		
/******************第二层 根据底盘行为状态机chassis_behaviour_mode来相应计算vx_set，vy_set，angle_set[local variables] ***********************************/

//				 如下6种模式
//				 CHASSIS_ZERO_FORCE,                  //底盘无力  遥控器掉线、开机初始化后
//  			 CHASSIS_NO_MOVE,                     //底盘保持不动
//  			 CHASSIS_NO_FOLLOW_YAW,               //底盘不跟随角度，角度是开环的，但前后左右是有速度环
		
   		chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

/******************第二层 根据底盘控制状态机chassis_move->chassis_mode最终得出chassis_move->vx_set|vy_set|wz_set期望 ***********************************/
//				 如下3种状态值
//  			 CHASSIS_VECTOR_NO_FOLLOW_YAW,				//底盘不随动
//  			 CHASSIS_VECTOR_RAW,									//原始底盘	
					
		/******************** 底盘控制状态机：底盘不随动 CHASSIS_VECTOR_NO_FOLLOW_YAW********************************************/		
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

        //放弃跟随云台
        //这个模式下，角度设置的为 角速度
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
	
}
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/****************** 5. chassis_control_loop *******************************************************************************************/

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //赋值电流值
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //raw控制直接返回
        return;
    }

    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //计算pid

    for (i = 0; i < 4; i++)
    {
        PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
