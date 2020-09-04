#include "gimbal_task.h"

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_speed_pid);                    \
                                                                                               \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_speed_pid);                  \
    }
		
#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
  */
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
		
Gimbal_System_t Gimbal_System;
int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0,Shoot_Can_Set_Current = 0;

//Main Task Function
static void Gimbal_Init(Gimbal_System_t *Gimbal_System);
static void Gimbal_Set_Mode(Gimbal_System_t *Gimbal_System);
static void Gimbal_Mode_Change_Save(Gimbal_System_t *Gimabl_System);
static void GImbal_Feedback_Update(Gimbal_System_t *Gimbal_System);
static void Gimbal_Set_Contorl(Gimbal_System_t *Gimbal_System);
static void Gimbal_Calculate(Gimbal_System_t *Gimbal_System);

//Tool Function:
static void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_System_t *gimbal_control_set);
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

		
void Gimbal_Setup(void)
{
		Gimbal_Init(&Gimbal_System);	
}


void Gimbal_Task(void)
{
		Gimbal_Set_Mode(&Gimbal_System);
		Gimbal_Mode_Change_Save(&Gimbal_System);
		GImbal_Feedback_Update(&Gimbal_System);
		Gimbal_Set_Contorl(&Gimbal_System);
		Gimbal_Calculate(&Gimbal_System);
	
#if YAW_TURN
		Yaw_Can_Set_Current = -Gimbal_System.gimbal_yaw_motor.given_current;
#else
		Yaw_Can_Set_Current = Gimbal_System.gimbal_yaw_motor.given_current;
#endif	
	
#if PITCH_TURN
		Pitch_Can_Set_Current = -Gimbal_System.gimbal_pitch_motor.given_current;
#else
		Pitch_Can_Set_Current = Gimbal_System.gimbal_pitch_motor.given_current;
#endif
	
}


static void Gimbal_Init(Gimbal_System_t *Gimbal_System)
{
	if(Gimbal_System == NULL)
	{
			return;
	}
	
	/*** YAW,PITCH的PID参数 ****/
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		
/**** 电机数据指针获取 ****/
			//motor_yaw结构体变量，存放Yaw云台电机的实时反馈
    Gimbal_System->gimbal_yaw_motor.gimbal_motor_measure = get_YAW_MOTOR_Measure_Point();		
			//motor_pit结构体变量，存放Pitch云台电机的实时反馈
		Gimbal_System->gimbal_pitch_motor.gimbal_motor_measure = get_PITCH_MOTOR_Measure_Point();
		
/**** 遥控器数据指针获取 ****/
		 //rc_ctrl结构体，存放遥控器实时通道值
    Gimbal_System->gimbal_rc_ctrl = get_remote_control_point();	
		
/**** 初始化电机模式 ****/	
    Gimbal_System->gimbal_yaw_motor.gimbal_motor_mode = Gimbal_System->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_STOP;
    Gimbal_System->gimbal_pitch_motor.gimbal_motor_mode = Gimbal_System->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_STOP;
		
/**** 初始化YAW和Pitch的PID 并清除所有PID输出值和期望、反馈 ****/		
		
		//初始化yaw电机相对角度环PID
    GIMBAL_PID_Init(&Gimbal_System->gimbal_yaw_motor.gimbal_motor_relative_angle_pid,\
		YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT,\
		YAW_ENCODE_RELATIVE_PID_KP,YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
		//初始化yaw电机角速度环PID
	  PID_Init(&Gimbal_System->gimbal_yaw_motor.gimbal_motor_speed_pid, PID_POSITION, Yaw_speed_pid,\
		YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    
		//初始化pitch电机相对角度环PID
    GIMBAL_PID_Init(&Gimbal_System->gimbal_pitch_motor.gimbal_motor_relative_angle_pid,\
		PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT,\
		PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
		//初始化pitch电机角速度环PID
    PID_Init(&Gimbal_System->gimbal_pitch_motor.gimbal_motor_speed_pid, PID_POSITION, Pitch_speed_pid,\
		PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);


    //清除云台PID的期望、反馈、输出值，全部清零，防止开机乱跑
    gimbal_total_pid_clear(Gimbal_System);
		
/**** 根据实际装配结果来设置偏差offset和最大最小限位 ****/
		//设置云台relative angle的offset,根据实际装配结果来调试
		Gimbal_System->gimbal_yaw_motor.offset_ecd 	 = GIMBAL_YAW_RELATIVE_ANGLE_OFFSET;    
		Gimbal_System->gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_RELATIVE_ANGLE_OFFSET;	
		
		//设置云台relative angle最大值和最小值 即机械限位最大值
		Gimbal_System->gimbal_yaw_motor.max_relative_angle = GIMBAL_YAW_MAX_RELATIVE_ANGLE;
		Gimbal_System->gimbal_yaw_motor.min_relative_angle = GIMBAL_YAW_MIN_RELATIVE_ANGLE;
		Gimbal_System->gimbal_pitch_motor.max_relative_angle = GIMBAL_PITCH_MAX_RELATIVE_ANGLE;
		Gimbal_System->gimbal_pitch_motor.min_relative_angle = GIMBAL_PITCH_MIN_RELATIVE_ANGLE;
/**** 更新yaw和pitch的absolute_angle绝对角度、relative_angle相对角度、motor_gyro角速度 ****/		
    GImbal_Feedback_Update(Gimbal_System);
		
		//更新YAW和PITCH期望为现在的反馈
    Gimbal_System->gimbal_yaw_motor.relative_angle_set = Gimbal_System->gimbal_yaw_motor.relative_angle;
    Gimbal_System->gimbal_pitch_motor.relative_angle_set = Gimbal_System->gimbal_pitch_motor.relative_angle;

}

static void Gimbal_Set_Mode(Gimbal_System_t *Gimbal_System)
{
		if(Gimbal_System == NULL)
		{
				return;
		}	
}
static void Gimbal_Mode_Change_Save(Gimbal_System_t *Gimbal_System)
{
		if(Gimbal_System == NULL)
		{
				return;
		}	
}
static void GImbal_Feedback_Update(Gimbal_System_t *Gimbal_System)
{
		if(Gimbal_System == NULL)
		{
				return;
		}
		
//////////////2006电机 

		static fp32 speed_filter[2][3] = {{ 0.0f,0.0f,0.0f },{ 0.0f,0.0f,0.0f }};

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
		
	  //二阶低通滤波	测速
		speed_filter[0][0] = speed_filter[0][1];
    speed_filter[0][1] = speed_filter[0][2];
    speed_filter[0][2] = speed_filter[0][1] * fliter_num[0] + speed_filter[0][0] * fliter_num[1] 
										 + (Gimbal_System->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm * Motor_RPM_TO_SPEED) * fliter_num[2];
    Gimbal_System->gimbal_yaw_motor.motor_speed = -speed_filter[0][2];

    speed_filter[1][0] = speed_filter[1][1];
    speed_filter[1][1] = speed_filter[1][2];
    speed_filter[1][2] = speed_filter[1][1] * fliter_num[0] + speed_filter[1][0] * fliter_num[1] 
										 + (Gimbal_System->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm * Motor_RPM_TO_SPEED) * fliter_num[2];
    Gimbal_System->gimbal_pitch_motor.motor_speed = -speed_filter[1][2];


    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (Gimbal_System->gimbal_pitch_motor.gimbal_motor_measure->ecd - Gimbal_System->gimbal_pitch_motor.gimbal_motor_measure->last_ecd > Half_ecd_range)
    {
        Gimbal_System->gimbal_pitch_motor.ecd_count--;
    }
    else if (Gimbal_System->gimbal_pitch_motor.gimbal_motor_measure->ecd - Gimbal_System->gimbal_pitch_motor.gimbal_motor_measure->last_ecd < -Half_ecd_range)
    {
        Gimbal_System->gimbal_pitch_motor.ecd_count++;
    }

    if (Gimbal_System->gimbal_pitch_motor.ecd_count == FULL_COUNT)
    {
        Gimbal_System->gimbal_pitch_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (Gimbal_System->gimbal_pitch_motor.ecd_count == -FULL_COUNT)
    {
        Gimbal_System->gimbal_pitch_motor.ecd_count = FULL_COUNT - 1;
    }
		
		    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (Gimbal_System->gimbal_yaw_motor.gimbal_motor_measure->ecd - Gimbal_System->gimbal_yaw_motor.gimbal_motor_measure->last_ecd > Half_ecd_range)
    {
        Gimbal_System->gimbal_yaw_motor.ecd_count--;
    }
    else if (Gimbal_System->gimbal_yaw_motor.gimbal_motor_measure->ecd - Gimbal_System->gimbal_yaw_motor.gimbal_motor_measure->last_ecd < -Half_ecd_range)
    {
        Gimbal_System->gimbal_yaw_motor.ecd_count++;
    }

    if (Gimbal_System->gimbal_yaw_motor.ecd_count == FULL_COUNT)
    {
        Gimbal_System->gimbal_yaw_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (Gimbal_System->gimbal_yaw_motor.ecd_count == -FULL_COUNT)
    {
        Gimbal_System->gimbal_yaw_motor.ecd_count = FULL_COUNT - 1;
    }


    //计算输出轴角度
    Gimbal_System->gimbal_pitch_motor.relative_angle = -(( Gimbal_System->gimbal_pitch_motor.ecd_count * ecd_range +  
																													 Gimbal_System->gimbal_pitch_motor.gimbal_motor_measure->ecd) * Motor_ECD_TO_ANGLE);
		
		Gimbal_System->gimbal_yaw_motor.relative_angle = -(( Gimbal_System->gimbal_yaw_motor.ecd_count * ecd_range +  
																													 Gimbal_System->gimbal_yaw_motor.gimbal_motor_measure->ecd) * Motor_ECD_TO_ANGLE);
}

static void Gimbal_Set_Contorl(Gimbal_System_t *Gimbal_System)
{
		if(Gimbal_System == NULL)
		{
				return;
		}	
		
		fp32 add_yaw_angle = 0.0f;		//yaw轴角度期望增量
    fp32 add_pitch_angle = 0.0f;	//pitch轴角度期望增量
				
		
    gimbal_mode_control_set(&add_yaw_angle, &add_pitch_angle, Gimbal_System);
		
/******************第二层 根据云台电机状态机的模式来给yaw和pitch角度期望relative_angle_set/absolute_angle_set限幅 *******************************************************/
	/****************** yaw电机角度期望relative_angle_set/absolute_angle_set限幅 ***************************************/
			if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_STOP)
			{
					//raw模式下，直接发送控制值
					gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
			}
			else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
			{
					//enconde模式下，电机编码角度控制
					GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, &add_yaw_angle);
			}
			
	/****************** pitch电机角度期望relative_angle_set/absolute_angle_set限幅 ***************************************/
			if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_STOP)
			{
					//raw模式下，直接发送控制值
					gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
			}
			else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
			{
					//enconde模式下，电机编码角度控制
					GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, &add_pitch_angle);
			}
}
static void Gimbal_Calculate(Gimbal_System_t *Gimbal_System)
{
		if(Gimbal_System == NULL)
		{
				return;
		}	
}

//Tool Function
static void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_System_t *gimbal_control_set)
{
		
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;
		
/********** 遥控器数据处理死区+配合鼠标x和y的值一起赋值给rc_add_yaw,rc_add_pit *******************************************************/
    //将gimbal_control->gimbal_rc_ctrl->rc.ch[YawChannel]处理死区， 赋值给yaw_channel		【static】
		//将gimbal_control->gimbal_rc_ctrl->rc.ch[PitchChannel]处理死区,赋值给pitch_channel 【static】
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);

		//yaw的增量 = k1倍的yaw_channel + k2倍的鼠标x方向值
		//pitch的增量 = k1倍的pitch_channel + k2倍的鼠标y方向值
    rc_add_yaw = yaw_channel * Yaw_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen;
    rc_add_pit = pitch_channel * Pitch_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * Pitch_Mouse_Sen;
		
/*********** 根据云台行为状态机gimbal_behaviour来采取相应控制，并将rc_add_yaw,rc_add_pit赋值进去得到add_yaw_angle和add_pitch_angle	***************/
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);	//掉头程序
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);	//return
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);			//rc_add_yaw、rc_add_pit置0
    }
    //将控制增加量赋值
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}
//pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}


