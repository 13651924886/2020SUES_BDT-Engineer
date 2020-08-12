/* TODO LIST*/
//1. Y MOTION添加编码器
//2. 串级PID写成struct简化Y_MOTION_System_PID_Task()和Z_MOTION_System_PID_Task()
//3. 二阶低通滤波写成struct 给3508速度滤波
//4. 3508 round_count改写成函数

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
static void XYZ_MOTION_mode_transit(XYZ_MOTION_System_t *XYZ_MOTION_move);
static void XYZ_MOTION_set_control(XYZ_MOTION_System_t *XYZ_MOTION_move);
static void XYZ_MOTION_control_loop(XYZ_MOTION_System_t *XYZ_MOTION_move);
		
static void Y_MOTION_System_PID_Task(Y_MOTION_System *Y_MOTION_move);
static void Z_MOTION_System_PID_Task(Z_MOTION_System *Z_MOTION_move);


void XYZ_MOTION_Setup(void)
{
	 XYZ_MOTION_init(&XYZ_MOTION_move);
}	

void XYZ_MOTION_task(void)
{
	 XYZ_MOTION_set_mode(&XYZ_MOTION_move);
	 XYZ_MOTION_mode_transit(&XYZ_MOTION_move);
	 XYZ_MOTION_feedback_update(&XYZ_MOTION_move);
	 XYZ_MOTION_set_control(&XYZ_MOTION_move);
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
		
		const static fp32 Z_MOTION_motor1_speed_pid[3] = {Y_MOTION_M3505_MOTOR_SPEED_PID_KP, Y_MOTION_M3505_MOTOR_SPEED_PID_KI, Y_MOTION_M3505_MOTOR_SPEED_PID_KD};
		const static fp32 Z_MOTION_motor2_speed_pid[3] = {Y_MOTION_M3505_MOTOR_SPEED_PID_KP, Y_MOTION_M3505_MOTOR_SPEED_PID_KI, Y_MOTION_M3505_MOTOR_SPEED_PID_KD};
		
    //位置环pid值
    const static fp32 Y_MOTION_motor_position_pid[3] = {Y_MOTION_M3505_MOTOR_POSITION_PID_KP,Y_MOTION_M3505_MOTOR_POSITION_PID_KI, Y_MOTION_M3505_MOTOR_POSITION_PID_KD};

		const static fp32 Z_MOTION_motor1_position_pid[3] = {Y_MOTION_M3505_MOTOR_POSITION_PID_KP,Y_MOTION_M3505_MOTOR_POSITION_PID_KI, Y_MOTION_M3505_MOTOR_POSITION_PID_KD};
		const static fp32 Z_MOTION_motor2_position_pid[3] = {Y_MOTION_M3505_MOTOR_POSITION_PID_KP,Y_MOTION_M3505_MOTOR_POSITION_PID_KI, Y_MOTION_M3505_MOTOR_POSITION_PID_KD};

    //开机状态为停止
    XYZ_MOTION_move_init->XYZ_mode = HOME;	

		
/**** 遥控器数据指针获取 ****/																											
    
    XYZ_MOTION_move_init->XYZ_MOTION_System_RC = get_remote_control_point();

/**** 电机数据指针获取 ****/
		
    XYZ_MOTION_move_init->Y_MOTION_System.Y_MOTION_motor.XYZ_MOTION_motor_measure = get_Y_MOTION_MOTOR_Measure_Point();
		XYZ_MOTION_move_init->Z_MOTION_System.Z_MOTION_motor1.XYZ_MOTION_motor_measure = get_Z_MOTION_MOTOR1_Measure_Point();
		XYZ_MOTION_move_init->Z_MOTION_System.Z_MOTION_motor2.XYZ_MOTION_motor_measure = get_Z_MOTION_MOTOR2_Measure_Point();
/**** 初始化PID 运动 ****/
		/* Y MOTION Motor*/
		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_System.Y_MOTION_motor_Speed_pid, PID_POSITION, Y_MOTION_motor_speed_pid,
							Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_IOUT);		
		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_System.Y_MOTION_motor_Position_pid, PID_POSITION, Y_MOTION_motor_position_pid,
							Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_IOUT);
							
		/* Z MOTION Motor1*/					
		PID_Init(&XYZ_MOTION_move_init->Z_MOTION_System.Z_MOTION_motor1_Position_pid, PID_POSITION, Z_MOTION_motor1_position_pid,
							Z_MOTION_M3505_MOTOR1_POSITION_PID_MAX_OUT, Z_MOTION_M3505_MOTOR1_POSITION_PID_MAX_IOUT);
		PID_Init(&XYZ_MOTION_move_init->Z_MOTION_System.Z_MOTION_motor1_Speed_pid, PID_POSITION, Z_MOTION_motor1_speed_pid,
							Z_MOTION_M3505_MOTOR1_SPEED_PID_MAX_OUT, Z_MOTION_M3505_MOTOR1_SPEED_PID_MAX_IOUT);	
							
		/* Z MOTION Motor2*/
		PID_Init(&XYZ_MOTION_move_init->Z_MOTION_System.Z_MOTION_motor2_Position_pid, PID_POSITION, Z_MOTION_motor2_position_pid,
							Z_MOTION_M3505_MOTOR2_POSITION_PID_MAX_OUT, Z_MOTION_M3505_MOTOR2_POSITION_PID_MAX_IOUT);
		PID_Init(&XYZ_MOTION_move_init->Z_MOTION_System.Z_MOTION_motor2_Speed_pid, PID_POSITION, Z_MOTION_motor2_speed_pid,
							Z_MOTION_M3505_MOTOR2_SPEED_PID_MAX_OUT, Z_MOTION_M3505_MOTOR2_SPEED_PID_MAX_IOUT);
						
 
//    ramp_init(&XYZ_MOTION_move_init->Y_MOTION_System.Y_cmd_ramp, Y_MOTION_CONTROL_TIME, Y_MOTION_MIN_POSITION,);

    //限幅和Y方向的运动速度【MAX and MIN】
		XYZ_MOTION_move_init->Y_MOTION_System.Position_Max = Y_MOTION_MAX_POSITION;
    XYZ_MOTION_move_init->Y_MOTION_System.Position_Min = Y_MOTION_MIN_POSITION;
		
		XYZ_MOTION_move_init->Z_MOTION_System.Position_Max = Z_MOTION_MAX_POSITION;
    XYZ_MOTION_move_init->Z_MOTION_System.Position_Min = Z_MOTION_MIN_POSITION;

    //更新一下数据
    XYZ_MOTION_feedback_update(XYZ_MOTION_move_init);
		
		
		XYZ_MOTION_move_init->X_MOTION_System.ControlFun = GPIO_CMD_Cylinder;
		
}


static void XYZ_MOTION_set_mode(XYZ_MOTION_System_t *XYZ_MOTION_move)
{
	 if (XYZ_MOTION_move == NULL)
    {
        return;
    }
	 if (switch_is_down(rc_ctrl.rc.s[0]))
		{
			XYZ_MOTION_move->XYZ_mode = STOP;
		}
	 if (switch_is_mid(rc_ctrl.rc.s[0]))
		{
			XYZ_MOTION_move->XYZ_mode = HOME;
		}
	 if (switch_is_up(rc_ctrl.rc.s[0]))
		{
			XYZ_MOTION_move->XYZ_mode = ENGAGE;
		}
		
}

static void XYZ_MOTION_mode_transit(XYZ_MOTION_System_t *XYZ_MOTION_move)
{
		if (XYZ_MOTION_move == NULL)
    {
        return;
    }
		
		if (XYZ_MOTION_move->last_XYZ_mode == XYZ_MOTION_move->XYZ_mode)
    {
        return;
    }
//		if ((XYZ_MOTION_move->last_XYZ_mode != HOME) && XYZ_MOTION_move->XYZ_mode == ENGAGE)
//    {
//        XYZ_MOTION_move->Y_MOTION_System.Position_set = 0.0f;
//    }
//		
		XYZ_MOTION_move->last_XYZ_mode = XYZ_MOTION_move->XYZ_mode;
}

static void XYZ_MOTION_feedback_update(XYZ_MOTION_System_t *XYZ_MOTION_move_update)
{
	   if (XYZ_MOTION_move_update == NULL)
    {
        return;
    }
		
		//////////////初次上电的ecd值，用于position归零
		static fp32 init_ecd = 0.0f;
		static fp32 init_ecd2 = 0.0f;
		static fp32 init_ecd3 = 0.0f;
		if(init_ecd == 0)init_ecd = XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.XYZ_MOTION_motor_measure->ecd;
		if(init_ecd2 == 0)init_ecd2 = XYZ_MOTION_move_update->Z_MOTION_System.Z_MOTION_motor1.XYZ_MOTION_motor_measure->ecd;
		if(init_ecd3 == 0)init_ecd3 = XYZ_MOTION_move_update->Z_MOTION_System.Z_MOTION_motor2.XYZ_MOTION_motor_measure->ecd;
		
		//////////////3508电机速度
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //////////////拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
		
    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] 
										 + (XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.XYZ_MOTION_motor_measure->speed_rpm * M3508_Motor_RPM_TO_SPEED) * fliter_num[2];
    XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.angular_speed = speed_fliter_3;

//    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.XYZ_MOTION_motor_measure->ecd - 
			XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.XYZ_MOTION_motor_measure->last_ecd > Half_ecd_range)
    {
        XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.round_count--;
    }
    else if (XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.XYZ_MOTION_motor_measure->ecd - 
			XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.XYZ_MOTION_motor_measure->last_ecd < -Half_ecd_range)
    {
        XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.round_count++;
    }
		
		XYZ_MOTION_move_update->Y_MOTION_System.Position = XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.round_count * ecd_range
																												+ XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.XYZ_MOTION_motor_measure->ecd
																												- init_ecd;

	/*********** 更新angular_speed和accel ***********/
        //更新电机反馈的实际速度，加速度是速度的PID微分
    XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.accel = XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor_Speed_pid.Dbuf[0]*Y_MOTION_CONTROL_FREQUENCE;

    

	/********** 根据更新的speed角速度来计算V线速度 ******************************/
		XYZ_MOTION_move_update->Y_MOTION_System.v = 2*PI*0.025f*(XYZ_MOTION_move_update->Y_MOTION_System.Y_MOTION_motor.angular_speed /60);
				
															

}
u8 testt = 0;
static void XYZ_MOTION_set_control(XYZ_MOTION_System_t *XYZ_MOTION_move)
{
	if (XYZ_MOTION_move == NULL)
  {
        return;
  }
		
	static fp32 Y_position_set = Y_MID_POSITION;	
	static fp32 Z_position_set = Z_INIT_POSITION;
	static uint16_t last_turn_keyboard = 0;
  static uint8_t 	mode_turn_flag = 0;
	
	/* 判断模式 */
	//HOME模式
	if (XYZ_MOTION_move->XYZ_mode ==	HOME)
	{
		Y_position_set = Y_MID_POSITION;
		Z_position_set = Z_INIT_POSITION;
	}
	//ENGAGE模式
	if (XYZ_MOTION_move->XYZ_mode ==	ENGAGE)
	{
		//短按一次检测
		if(	((XYZ_MOTION_move->XYZ_MOTION_System_RC->key.v & Y_MOTION_RIGHT_KEY) && !(last_turn_keyboard & Y_MOTION_RIGHT_KEY))||
			  ((XYZ_MOTION_move->XYZ_MOTION_System_RC->key.v & Y_MOTION_LEFT_KEY)  && !(last_turn_keyboard & Y_MOTION_LEFT_KEY)) ||
				((XYZ_MOTION_move->XYZ_MOTION_System_RC->key.v & Z_MOTION_KEY)  && !(last_turn_keyboard & Z_MOTION_KEY)) )
		{
				 if (mode_turn_flag == 0)
				 {
						mode_turn_flag = 1;		
						testt = 1;
				 }
		}
		
	  last_turn_keyboard = XYZ_MOTION_move->XYZ_MOTION_System_RC->key.v ;
		
	  if (mode_turn_flag)
	  {
				static u8 i = 0;
				if(XYZ_MOTION_move->XYZ_MOTION_System_RC->key.v & Y_MOTION_LEFT_KEY && i == 0 )
				{
						Y_position_set +=  Y_POSITION_ADD;
						Y_position_set = fp32_constrain(Y_position_set,XYZ_MOTION_move->Y_MOTION_System.Position_Min, XYZ_MOTION_move->Y_MOTION_System.Position_Max);
						i++;
				}
				if(XYZ_MOTION_move->XYZ_MOTION_System_RC->key.v & Y_MOTION_RIGHT_KEY && i == 0)	
				{
						Y_position_set -=  Y_POSITION_ADD;
						Y_position_set = fp32_constrain(Y_position_set,XYZ_MOTION_move->Y_MOTION_System.Position_Min, XYZ_MOTION_move->Y_MOTION_System.Position_Max);
						i++;
				}
				if(XYZ_MOTION_move->XYZ_MOTION_System_RC->key.v & Z_MOTION_KEY && i == 0)	
				{
						if( Z_position_set == Z_INIT_POSITION)
						{
							Z_position_set +=  Z_POSITION_ADD;
							Z_position_set = fp32_constrain(Z_position_set,XYZ_MOTION_move->Z_MOTION_System.Position_Min, XYZ_MOTION_move->Z_MOTION_System.Position_Max);
						}
						if( Z_position_set != Z_INIT_POSITION)
						{
							Z_position_set -=  Z_POSITION_ADD;
							Z_position_set = fp32_constrain(Z_position_set,XYZ_MOTION_move->Z_MOTION_System.Position_Min, XYZ_MOTION_move->Z_MOTION_System.Position_Max);
						}
						i++;
				}
				i = 0;
				mode_turn_flag = 0;
		}		
	}
	XYZ_MOTION_move->Y_MOTION_System.Position_set = Y_position_set;
	XYZ_MOTION_move->Z_MOTION_System.Position_set = Z_position_set;
	
}

static void XYZ_MOTION_control_loop(XYZ_MOTION_System_t *XYZ_MOTION_move)
{
	  if (XYZ_MOTION_move == NULL)
    {
        return;
    }
		Y_MOTION_System_PID_Task(&XYZ_MOTION_move->Y_MOTION_System);
		Z_MOTION_System_PID_Task(&XYZ_MOTION_move->Z_MOTION_System);
		
		CAN1_CMD_XYZ(XYZ_MOTION_move->Y_MOTION_System.Y_MOTION_motor.give_current, XYZ_MOTION_move->Z_MOTION_System.Z_MOTION_motor1.give_current,
                                XYZ_MOTION_move->Z_MOTION_System.Z_MOTION_motor2.give_current, 0);		
}

static void Y_MOTION_System_PID_Task(Y_MOTION_System *Y_MOTION_move)
{
			//角度环外环	位置式
		Y_MOTION_move->Y_MOTION_motor.speed_set = PID_Calc(&Y_MOTION_move->Y_MOTION_motor_Position_pid,
																	Y_MOTION_move->Position, Y_MOTION_move->Position_set);
		//角速度环内环 位置式
		Y_MOTION_move->Y_MOTION_motor.current_set = PID_Calc(&Y_MOTION_move->Y_MOTION_motor_Speed_pid,
																	Y_MOTION_move->Y_MOTION_motor.angular_speed,Y_MOTION_move->Y_MOTION_motor.speed_set);
    //控制值赋值
    Y_MOTION_move->Y_MOTION_motor.give_current = (int16_t)(Y_MOTION_move->Y_MOTION_motor.current_set);
}

static void Z_MOTION_System_PID_Task(Z_MOTION_System *Z_MOTION_move)
{
				//角度环外环	位置式
		Z_MOTION_move->Z_MOTION_motor1.speed_set = PID_Calc(&Z_MOTION_move->Z_MOTION_motor1_Position_pid,
																	Z_MOTION_move->Position, Z_MOTION_move->Position_set);
		//角速度环内环 位置式
		Z_MOTION_move->Z_MOTION_motor1.current_set = PID_Calc(&Z_MOTION_move->Z_MOTION_motor1_Speed_pid,
																	Z_MOTION_move->Z_MOTION_motor1.angular_speed,Z_MOTION_move->Z_MOTION_motor1.speed_set);
    //控制值赋值
    Z_MOTION_move->Z_MOTION_motor1.give_current = (int16_t)(Z_MOTION_move->Z_MOTION_motor1.current_set);
	
					//角度环外环	位置式
		Z_MOTION_move->Z_MOTION_motor2.speed_set = PID_Calc(&Z_MOTION_move->Z_MOTION_motor2_Position_pid,
																	Z_MOTION_move->Position, Z_MOTION_move->Position_set);
		//角速度环内环 位置式
		Z_MOTION_move->Z_MOTION_motor2.current_set = PID_Calc(&Z_MOTION_move->Z_MOTION_motor2_Speed_pid,
																	Z_MOTION_move->Z_MOTION_motor2.angular_speed,Z_MOTION_move->Z_MOTION_motor2.speed_set);
    //控制值赋值
    Z_MOTION_move->Z_MOTION_motor2.give_current = (int16_t)(Z_MOTION_move->Z_MOTION_motor2.current_set);
}
