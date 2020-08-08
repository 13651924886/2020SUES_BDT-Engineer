#include "XYZ_MOTION_task.h"

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

XYZ_MOTION_System_t XYZ_MOTION_move;

static void XYZ_MOTION_init(XYZ_MOTION_System_t *XYZ_MOTION_move_init);
static void XYZ_MOTION_feedback_update(XYZ_MOTION_System_t *XYZ_MOTION_move_update);
static void XYZ_MOTION_set_mode(XYZ_MOTION_System_t *XYZ_MOTION_move);
static void XYZ_MOTION_set_contorl(XYZ_MOTION_System_t *XYZ_MOTION_move);
static void XYZ_MOTION_control_loop(XYZ_MOTION_System_t *XYZ_MOTION_move);
static void XYZ_MOTION_rc_to_motor_speed(const fp32 vy_set, fp32 *Motor_Speed);

void XYZ_MOTION_Setup(void)
{
	 XYZ_MOTION_init(&XYZ_MOTION_move);
}	

void XYZ_MOTION_task(void)
{
	 XYZ_MOTION_set_mode(&XYZ_MOTION_move);
	 XYZ_MOTION_feedback_update(&XYZ_MOTION_move);
	 XYZ_MOTION_set_contorl(&XYZ_MOTION_move);
	 XYZ_MOTION_control_loop(&XYZ_MOTION_move);
}	
		
		
static void XYZ_MOTION_init(XYZ_MOTION_System_t *XYZ_MOTION_move_init)
{
    if (XYZ_MOTION_move_init == NULL)
    {
        return;
    }

    //速度环pid值
    const static fp32 Y_MOTION_motor_speed_pid[3] = {Y_MOTION_M3505_MOTOR_SPEED_PID_KP, Y_MOTION_M3505_MOTOR_SPEED_PID_KI, Y_MOTION_M3505_MOTOR_SPEED_PID_KD};
    //位置环pid值
    const static fp32 Y_MOTION_motor_position_pid[3] = {Y_MOTION_M3505_MOTOR_POSITION_PID_KP,Y_MOTION_M3505_MOTOR_POSITION_PID_KI, Y_MOTION_M3505_MOTOR_POSITION_PID_KD};
    //底盘一阶滤波参数
    const static fp32 chassis_y_order_filter[1] = {MOTION_ACCEL_Y_NUM};

    //底盘开机状态为停止
    XYZ_MOTION_move_init->Y_MOTION_mode = Y_MOTION_STOP;	
/**** 遥控器数据指针获取 ****/																											
    
    XYZ_MOTION_move_init->Y_MOTION_System_RC = get_remote_control_point();

/**** 电机数据指针获取 ****/
		
    XYZ_MOTION_move_init->Y_MOTION_Left_motor.Y_MOTION_motor_measure = get_Y_LEFT_MOTOR_Measure_Point();
		XYZ_MOTION_move_init->Y_MOTION_Right_motor.Y_MOTION_motor_measure = get_Y_RIGHT_MOTOR_Measure_Point();

/**** 初始化PID 运动 ****/
	
		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_Left_motor_Speed_pid, PID_POSITION, Y_MOTION_motor_speed_pid,
							Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_IOUT);
		
		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_Left_motor_Position_pid, PID_POSITION, Y_MOTION_motor_position_pid,
							Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_IOUT);
							
		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_Right_motor_Speed_pid, PID_POSITION, Y_MOTION_motor_speed_pid,
							Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_IOUT);

		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_Left_motor_Position_pid, PID_POSITION, Y_MOTION_motor_position_pid,
							Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_IOUT);

   
		//用一阶滤波代替斜波函数生成
		//初始化后才可以使用一阶低通滤波
    first_order_filter_init(&XYZ_MOTION_move_init->chassis_cmd_slow_set_vy, Y_MOTION_CONTROL_TIME, chassis_y_order_filter);

    //限幅和Y方向的运动速度【MAX and MIN】
    XYZ_MOTION_move_init->vy_max_speed = Y_MOTION_MAX_SPEED;
    XYZ_MOTION_move_init->vy_min_speed = -Y_MOTION_MAX_SPEED;

    //更新一下数据
    //chassis_feedback_update(XYZ_MOTION_move_init);
}

static void XYZ_MOTION_set_mode(XYZ_MOTION_System_t *XYZ_MOTION_move)
{
	 if (XYZ_MOTION_move == NULL)
    {
        return;
    }
//	 if (switch_is_up(rc_ctrl.rc.s[0]))
//		{
//			XYZ_MOTION_move->Y_MOTION_mode = Y_MOTION_STOP;
//		}
//	 if (switch_is_mid(rc_ctrl.rc.s[0]))
//		{
//			XYZ_MOTION_move->Y_MOTION_mode = Y_MOTION_ENGAGE;
//		}
//	 if(switch_is_down(rc_ctrl.rc.s[0]))
//		{
//			XYZ_MOTION_move->Y_MOTION_mode = Y_MOTION_STOP;
//		}
		
		
//    static uint16_t last_turn_keyboard = 0;
//    static uint8_t mode_turn_flag = 0;
//		if(XYZ_MOTION_move->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT )
//		{
//				XYZ_MOTION_move->super_cap.discharge_cmd_state = Discharging;
//		}
//		else
//		{
//				XYZ_MOTION_move->super_cap.discharge_cmd_state = Waiting;
//		}
//			
//		if(	(XYZ_MOTION_move->chassis_RC->key.v & KEY_PRESSED_OFFSET_C) && !(last_turn_keyboard & KEY_PRESSED_OFFSET_C))
//		{
//				 if (mode_turn_flag == 0)
//				 {
//						mode_turn_flag = 1;								
//				 }
//		}
//		
//	  last_turn_keyboard = XYZ_MOTION_move->chassis_RC->key.v ;
//		
//	  if (mode_turn_flag)
//	  {
//				static u8 i = 0;
//				if(XYZ_MOTION_move->super_cap.charge_cmd_state == Charging && i == 0 )
//				{
//						XYZ_MOTION_move->super_cap.charge_cmd_state = Waiting;
//						i++;
//				}
//				if(XYZ_MOTION_move->super_cap.charge_cmd_state == Waiting && i == 0)	
//				{
//						XYZ_MOTION_move->super_cap.charge_cmd_state = Charging;
//						i++;
//				}
//				i = 0;
//				mode_turn_flag = 0;
//		}
}

static void XYZ_MOTION_feedback_update(XYZ_MOTION_System_t *XYZ_MOTION_move_update)
{
	   if (XYZ_MOTION_move_update == NULL)
    {
        return;
    }
	/*********** 更新chassis_move->motor_chassis[i].speed和accel ***********/
        //更新电机反馈的实际速度，加速度是速度的PID微分
				XYZ_MOTION_move_update->Y_MOTION_Left_motor.speed = XYZ_MOTION_move_update->Y_MOTION_Left_motor.Y_MOTION_motor_measure->speed_rpm;
				XYZ_MOTION_move_update->Y_MOTION_Right_motor.speed = XYZ_MOTION_move_update->Y_MOTION_Right_motor.Y_MOTION_motor_measure->speed_rpm;
																								
        XYZ_MOTION_move_update->Y_MOTION_Left_motor.accel = XYZ_MOTION_move_update->Y_MOTION_Left_motor_Speed_pid.Dbuf[0]*Y_MOTION_CONTROL_FREQUENCE;
		    XYZ_MOTION_move_update->Y_MOTION_Right_motor.accel = XYZ_MOTION_move_update->Y_MOTION_Right_motor_Speed_pid.Dbuf[0]*Y_MOTION_CONTROL_FREQUENCE;

    

	/********** 根据更新的chassis_move->motor_chassis[i].speed来计算Vx，Vy，Wz ******************************/
				XYZ_MOTION_move_update->vy = 2*PI*0.025f*(XYZ_MOTION_move_update->Y_MOTION_Left_motor.speed /60);
															

}

static void XYZ_MOTION_set_contorl(XYZ_MOTION_System_t *XYZ_MOTION_move)
{
	if (XYZ_MOTION_move == NULL)
  {
        return;
  }
		
	fp32 vy_set = 0.0f, position_set = 0.0f;
	
	XYZ_MOTION_rc_filter(&vy_set, XYZ_MOTION_move);
	
	
	if (XYZ_MOTION_move->Y_MOTION_mode ==	Y_MOTION_STOP)
	{
		XYZ_MOTION_move->vy_set = 0.0f;
	}
	if (XYZ_MOTION_move->Y_MOTION_mode ==	Y_MOTION_ENGAGE)
	{
		XYZ_MOTION_move->vy_set = fp32_constrain(vy_set, 
							XYZ_MOTION_move->vy_min_speed, XYZ_MOTION_move->vy_max_speed);
	}
}

static void XYZ_MOTION_control_loop(XYZ_MOTION_System_t *XYZ_MOTION_move)
{
    fp32 motor_speed = 0.0f;

	//将遥控器通道打杆值vy_set(m/s) 转换成 电机期望转速rpm
    XYZ_MOTION_rc_to_motor_speed(XYZ_MOTION_move->vy_set, &motor_speed);

    if (XYZ_MOTION_move->Y_MOTION_mode == Y_MOTION_STOP)
    {
        //赋值电流值
       XYZ_MOTION_move->Y_MOTION_Left_motor.give_current = 0.0f;
			 XYZ_MOTION_move->Y_MOTION_Right_motor.give_current = 0.0f;

        //raw控制直接返回
        return;
    }

    //计算轮子控制最大速度，并限制其最大速度
//		if( motor_speed < 0)
//		{
//			if(motor_speed < -Y_MOTION_MAX_SPEED)
//			{
//				motor_speed = -Y_MOTION_MAX_SPEED;
//			}
//		}
//		else if( motor_speed >= 0 )
//		{
//				if(motor_speed > Y_MOTION_MAX_SPEED)
//			{
//				motor_speed = Y_MOTION_MAX_SPEED;
//			}
//		}
		
		XYZ_MOTION_move->Y_MOTION_Left_motor.speed_set = -motor_speed;
		XYZ_MOTION_move->Y_MOTION_Right_motor.speed_set = motor_speed;

    //计算pid
    PID_Calc(&XYZ_MOTION_move->Y_MOTION_Left_motor_Speed_pid, 
		XYZ_MOTION_move->Y_MOTION_Left_motor.speed, XYZ_MOTION_move->Y_MOTION_Left_motor.speed_set);
		PID_Calc(&XYZ_MOTION_move->Y_MOTION_Right_motor_Speed_pid, 
		XYZ_MOTION_move->Y_MOTION_Right_motor.speed, XYZ_MOTION_move->Y_MOTION_Right_motor.speed_set);

    //赋值电流值
		XYZ_MOTION_move->Y_MOTION_Left_motor.give_current= (int16_t)(XYZ_MOTION_move->Y_MOTION_Left_motor_Speed_pid.out);
		XYZ_MOTION_move->Y_MOTION_Right_motor.give_current= (int16_t)(XYZ_MOTION_move->Y_MOTION_Right_motor_Speed_pid.out);

}

static void XYZ_MOTION_rc_to_motor_speed(const fp32 vy_set, fp32 *Motor_Speed)
{
 //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    *Motor_Speed = 9.6f * (1/SychronicWheelRadius) * vy_set;   // V(m/s)=2 * PI * R(m) * N(Round Per Second)
 
}


void XYZ_MOTION_rc_filter(fp32 *vy_set, XYZ_MOTION_System_t *XYZ_MOTION_move)
{
    if (XYZ_MOTION_move == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vy_channel;
    fp32 vy_set_channel;
		
//死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(XYZ_MOTION_move->Y_MOTION_System_RC->rc.ch[Y_MOTION_CONTROL_CHANNEL], vy_channel, Y_MOTION_RC_DEADLINE);

    vy_set_channel = vy_channel * -Y_MOTION_RC_SEN;

//    if (XYZ_MOTION_move->Y_MOTION_System_RC->key.v & CHASSIS_FRONT_KEY)
//    {
//        vx_set_channel = KEYBOARD_CHASSIS_SPEED_X;
//    }
//    else if (XYZ_MOTION_move->Y_MOTION_System_RC->key.v & CHASSIS_BACK_KEY)
//    {
//        vx_set_channel = -KEYBOARD_CHASSIS_SPEED_X;
//    }

//    if (XYZ_MOTION_move->Y_MOTION_System_RC->key.v & CHASSIS_LEFT_KEY)
//    {
//        vy_set_channel = KEYBOARD_CHASSIS_SPEED_Y;
//    }
//    else if (XYZ_MOTION_move->Y_MOTION_System_RC->key.v & CHASSIS_RIGHT_KEY)
//    {
//        vy_set_channel = -KEYBOARD_CHASSIS_SPEED_Y;
//    }

//一阶低通滤波代替斜波作为底盘速度输入
		// In:vx_set_channel   Out:chassis_move->chassis_cmd_slow_set_vx
		// In:vy_set_channel   Out:chassis_move->chassis_cmd_slow_set_vy
    first_order_filter_cali(&XYZ_MOTION_move->chassis_cmd_slow_set_vy, vy_set_channel);

//停止信号，不需要缓慢加速，直接减速到零
		//拨杆回中立即速度为0
    if (vy_set_channel < Y_MOTION_RC_DEADLINE * Y_MOTION_RC_SEN && vy_set_channel > -Y_MOTION_RC_DEADLINE * Y_MOTION_RC_SEN)
    {
        XYZ_MOTION_move->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		
    *vy_set = XYZ_MOTION_move->chassis_cmd_slow_set_vy.out;
}



