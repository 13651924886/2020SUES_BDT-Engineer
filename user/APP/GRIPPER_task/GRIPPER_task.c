#include "GRIPPER_task.h"

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

extern RC_ctrl_t rc_ctrl;

GRIPPER_System_t GRIPPER_move;

static void GRIPPER_init(GRIPPER_System_t *GRIPPER_move_init);
static void GRIPPER_feedback_update(GRIPPER_System_t *GRIPPER_move_update);
static void GRIPPER_set_mode(GRIPPER_System_t *GRIPPER_mode);
static void GRIPPER_set_contorl(GRIPPER_System_t *GRIPPER_move);
static void GRIPPER_control_loop(GRIPPER_System_t *GRIPPER_control_loop);
static void GRIPPER_AngularVelocity_rc_to_motor_speed(const fp32 angular_velocity_set, fp32 *Motor_Speed);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
static fp32 motor_ecd_to_position_change(GRIPPER_System_t *Gripper_ecd, uint16_t *position  );


void GRIPPER_Setup(void)
{
	GRIPPER_init(&GRIPPER_move);
}

void GRIPPER_task(void)
{
	GRIPPER_set_mode(&GRIPPER_move);
	GRIPPER_feedback_update(&GRIPPER_move);
	GRIPPER_set_contorl(&GRIPPER_move);
	GRIPPER_control_loop(&GRIPPER_move);
			
}

void GRIPPER_rc_filter(fp32 *Angular_Velocity_set, GRIPPER_System_t *GRIPPER_move)
{
    if (GRIPPER_move == NULL || Angular_Velocity_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t angular_velocity_channel;
    fp32 angular_velocity_set_channel;
		
//死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(GRIPPER_move->GRIPPER_System_RC->rc.ch[GRIPPER_CONTROL_CHANNEL], angular_velocity_channel, GRIPPER_RC_DEADLINE);

    angular_velocity_set_channel = angular_velocity_channel * -GRIPPER_RC_SEN;

//    if (GRIPPER_move->GRIPPER_System_RC->key.v & CHASSIS_FRONT_KEY)
//    {
//        vx_set_channel = KEYBOARD_CHASSIS_SPEED_X;
//    }
//    else if (GRIPPER_move->GRIPPER_System_RC->key.v & CHASSIS_BACK_KEY)
//    {
//        vx_set_channel = -KEYBOARD_CHASSIS_SPEED_X;
//    }

//    if (GRIPPER_move->GRIPPER_System_RC->key.v & CHASSIS_1_KEY)
//    {
//        vy_set_channel = KEYBOARD_CHASSIS_SPEED_Y;
//    }
//    else if (GRIPPER_move->GRIPPER_System_RC->key.v & CHASSIS_2_KEY)
//    {
//        vy_set_channel = -KEYBOARD_CHASSIS_SPEED_Y;
//    }

//一阶低通滤波代替斜波作为底盘速度输入
		// In:vx_set_channel   Out:chassis_move->chassis_cmd_slow_set_vx
		// In:vy_set_channel   Out:chassis_move->chassis_cmd_slow_set_vy
    first_order_filter_cali(&GRIPPER_move->Gripper_cmd_slow_set_Angular_Velocity, angular_velocity_set_channel);

//停止信号，不需要缓慢加速，直接减速到零
		//拨杆回中立即速度为0
    if (angular_velocity_set_channel < GRIPPER_RC_DEADLINE * GRIPPER_RC_SEN && angular_velocity_set_channel > -GRIPPER_RC_DEADLINE * GRIPPER_RC_SEN)
    {
        GRIPPER_move->Gripper_cmd_slow_set_Angular_Velocity.out = 0.0f;
    }
		
    *Angular_Velocity_set = GRIPPER_move->Gripper_cmd_slow_set_Angular_Velocity.out;
}

static void GRIPPER_init(GRIPPER_System_t *GRIPPER_move_init)
{
    if (GRIPPER_move_init == NULL)
    {
        return;
    }

    //速度环pid值
    const static fp32 GRIPPER_motor_angular_speed_pid[3] = {GRIPPER_M3505_MOTOR_SPEED_PID_KP, GRIPPER_M3505_MOTOR_SPEED_PID_KI, GRIPPER_M3505_MOTOR_SPEED_PID_KD};
    //位置环pid值
    const static fp32 GRIPPER_motor_position_pid[3] = {GRIPPER_M3505_MOTOR_POSITION_PID_KP,GRIPPER_M3505_MOTOR_POSITION_PID_KI, GRIPPER_M3505_MOTOR_POSITION_PID_KD};
    //底盘一阶滤波参数
    const static fp32 Gripper_order_filter[1] = {MOTION_ACCEL_Y_NUM};

    //底盘开机状态为停止
    GRIPPER_move_init->GRIPPER_mode = GRIPPER_STOP;	
/**** 遥控器数据指针获取 ****/																											
    
    GRIPPER_move_init->GRIPPER_System_RC = get_remote_control_point();

/**** 电机数据指针获取 ****/
		
    GRIPPER_move_init->GRIPPER_1_motor.gripper_motor_measure = get_GRIPPER_1_MOTOR_Measure_Point();
		GRIPPER_move_init->GRIPPER_2_motor.gripper_motor_measure = get_GRIPPER_2_MOTOR_Measure_Point();
		
/**** 电机OFFSET获取,用于计算Relative Angle****/
		
		GRIPPER_move_init->GRIPPER_1_motor.offset_ecd = GRIPPER_MOTOR1_RELATIVE_ANGLE_OFFSET;
		GRIPPER_move_init->GRIPPER_2_motor.offset_ecd = GRIPPER_MOTOR2_RELATIVE_ANGLE_OFFSET;

/**** 初始化PID 运动 ****/
	
		PID_Init(&GRIPPER_move_init->GRIPPER_1_motor_Angular_Speed_pid, PID_POSITION, GRIPPER_motor_angular_speed_pid,
							GRIPPER_M3505_MOTOR_SPEED_PID_MAX_OUT, GRIPPER_M3505_MOTOR_SPEED_PID_MAX_IOUT);
		
		PID_Init(&GRIPPER_move_init->GRIPPER_1_motor_Position_pid, PID_POSITION, GRIPPER_motor_position_pid,
							GRIPPER_M3505_MOTOR_POSITION_PID_MAX_OUT, GRIPPER_M3505_MOTOR_POSITION_PID_MAX_IOUT);
							
		PID_Init(&GRIPPER_move_init->GRIPPER_2_motor_Angular_Speed_pid, PID_POSITION, GRIPPER_motor_angular_speed_pid,
							GRIPPER_M3505_MOTOR_SPEED_PID_MAX_OUT, GRIPPER_M3505_MOTOR_SPEED_PID_MAX_IOUT);

		PID_Init(&GRIPPER_move_init->GRIPPER_1_motor_Position_pid, PID_POSITION, GRIPPER_motor_position_pid,
							GRIPPER_M3505_MOTOR_POSITION_PID_MAX_OUT, GRIPPER_M3505_MOTOR_POSITION_PID_MAX_IOUT);

   
		//用一阶滤波代替斜波函数生成
		//初始化后才可以使用一阶低通滤波
    first_order_filter_init(&GRIPPER_move_init->Gripper_cmd_slow_set_Angular_Velocity, GRIPPER_CONTROL_TIME, Gripper_order_filter);

    //限幅和Y方向的运动速度【MAX and MIN】
    GRIPPER_move_init->Angular_Velocity_max_speed = GRIPPER_MAX_SPEED;
    GRIPPER_move_init->Angular_Velocity_min_speed = -GRIPPER_MAX_SPEED;
		
		//目前的爪子相对角度位置由Motor1的相对角度来决定
		GRIPPER_move_init->Position_Max = GRIPPER_MAX_POSITION;
		GRIPPER_move_init->Position_Min = GRIPPER_MIN_POSITION;

    //更新一下数据
    GRIPPER_feedback_update(GRIPPER_move_init);
}
static void GRIPPER_set_mode(GRIPPER_System_t *GRIPPER_move)
{
	 if (GRIPPER_move == NULL)
    {
        return;
    }
	 if (switch_is_up(rc_ctrl.rc.s[0]))
		{
			GRIPPER_move->GRIPPER_mode = GRIPPER_STOP;
		}
	 if (switch_is_mid(rc_ctrl.rc.s[0]))
		{
			GRIPPER_move->GRIPPER_mode = GRIPPER_ENGAGE;
		}
	 if(switch_is_down(rc_ctrl.rc.s[0]))
		{
			GRIPPER_move->GRIPPER_mode = GRIPPER_STOP;
		}
}

static void GRIPPER_feedback_update(GRIPPER_System_t *GRIPPER_move_update)
{
	   if (GRIPPER_move_update == NULL)
    {
        return;
    }
	/*********** 更新chassis_move->motor_chassis[i].speed和accel ***********/
        //更新电机反馈的实际速度，加速度是速度的PID微分
				GRIPPER_move_update->GRIPPER_1_motor.speed = GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->speed_rpm;
				GRIPPER_move_update->GRIPPER_2_motor.speed = GRIPPER_move_update->GRIPPER_2_motor.gripper_motor_measure->speed_rpm;
																								
        GRIPPER_move_update->GRIPPER_1_motor.accel = GRIPPER_move_update->GRIPPER_1_motor_Angular_Speed_pid.Dbuf[0]*GRIPPER_CONTROL_FREQUENCE;
		    GRIPPER_move_update->GRIPPER_2_motor.accel = GRIPPER_move_update->GRIPPER_2_motor_Angular_Speed_pid.Dbuf[0]*GRIPPER_CONTROL_FREQUENCE;
				
				GRIPPER_move_update->GRIPPER_1_motor.relative_angle = motor_ecd_to_angle_change(GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->ecd,GRIPPER_move_update->GRIPPER_1_motor.offset_ecd);
				GRIPPER_move_update->GRIPPER_2_motor.relative_angle = motor_ecd_to_angle_change(GRIPPER_move_update->GRIPPER_2_motor.gripper_motor_measure->ecd,GRIPPER_move_update->GRIPPER_2_motor.offset_ecd);

				//目前抓手位置由Motor1的相对角度来决定
				GRIPPER_move_update->Position = motor_ecd_to_angle_change(GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->ecd,GRIPPER_move_update->GRIPPER_1_motor.offset_ecd);
		
	/********** 根据更新的chassis_move->motor_chassis[i].speed来计算Vx，Vy，Wz ******************************/
				//目前抓手角速度由Motor1的角速度来决定
				GRIPPER_move_update->Angular_Velocity = GRIPPER_move_update->GRIPPER_1_motor.speed; //角速度实际上并不是RPM转速，之后再改
				//目前抓手位置由Motor1的相对角度来决定
				GRIPPER_move_update->Position = motor_ecd_to_angle_change(GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->ecd,GRIPPER_move_update->GRIPPER_1_motor.offset_ecd);				

}

static void GRIPPER_set_contorl(GRIPPER_System_t *GRIPPER_move)
{
	if (GRIPPER_move == NULL)
  {
        return;
  }
		
	fp32 angular_velocity_set = 0.0f, position_set = 0.0f;
	
	GRIPPER_rc_filter(&angular_velocity_set, GRIPPER_move);
	
	
	if (GRIPPER_move->GRIPPER_mode ==	GRIPPER_STOP)
	{
		GRIPPER_move->Angular_Velocity_set = 0.0f;
	}
	if (GRIPPER_move->GRIPPER_mode ==	GRIPPER_ENGAGE)
	{
		GRIPPER_move->Angular_Velocity_set = fp32_constrain(angular_velocity_set, 
							GRIPPER_move->Angular_Velocity_min_speed, GRIPPER_move->Angular_Velocity_max_speed);
	}
}

static void GRIPPER_control_loop(GRIPPER_System_t *GRIPPER_move)
{
    fp32 motor_speed = 0.0f;

	//将遥控器通道打杆值Angular_Velocity_set(度/s) 转换成 电机期望转速rpm
    GRIPPER_AngularVelocity_rc_to_motor_speed(GRIPPER_move->Angular_Velocity_set, &motor_speed);

    if (GRIPPER_move->GRIPPER_mode == GRIPPER_STOP)
    {
        //赋值电流值
       GRIPPER_move->GRIPPER_1_motor.give_current = 0.0f;
			 GRIPPER_move->GRIPPER_2_motor.give_current = 0.0f;

        //raw控制直接返回
        return;
    }
//		}
		
		GRIPPER_move->GRIPPER_1_motor.speed_set = -motor_speed;
		GRIPPER_move->GRIPPER_2_motor.speed_set = motor_speed;

    //计算pid
    PID_Calc(&GRIPPER_move->GRIPPER_1_motor_Angular_Speed_pid, 
		GRIPPER_move->GRIPPER_1_motor.speed, GRIPPER_move->GRIPPER_1_motor.speed_set);
		PID_Calc(&GRIPPER_move->GRIPPER_2_motor_Angular_Speed_pid, 
		GRIPPER_move->GRIPPER_2_motor.speed, GRIPPER_move->GRIPPER_2_motor.speed_set);

    //赋值电流值
		GRIPPER_move->GRIPPER_1_motor.give_current= (int16_t)(GRIPPER_move->GRIPPER_1_motor_Angular_Speed_pid.out);
		GRIPPER_move->GRIPPER_2_motor.give_current= (int16_t)(GRIPPER_move->GRIPPER_2_motor_Angular_Speed_pid.out);

}

static void GRIPPER_AngularVelocity_rc_to_motor_speed(const fp32 angular_velocity_set, fp32 *Motor_Speed)
{
    *Motor_Speed =  angular_velocity_set; 
 
}

static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range) ///电机码盘值中值Half_ecd_range = 4096
    {
        relative_ecd -= ecd_range;		///电机码盘值最大值ecd_range = 8191
    }
    else if (relative_ecd < -Half_ecd_range)//-Half_ecd_range = -4096
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;//电机编码值转化成角度值 rad
}

static fp32 motor_ecd_to_position_change(GRIPPER_System_t *Gripper_ecd, uint16_t *po  );

