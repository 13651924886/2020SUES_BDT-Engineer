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

    //�ٶȻ�pidֵ
    const static fp32 Y_MOTION_motor_speed_pid[3] = {Y_MOTION_M3505_MOTOR_SPEED_PID_KP, Y_MOTION_M3505_MOTOR_SPEED_PID_KI, Y_MOTION_M3505_MOTOR_SPEED_PID_KD};
    //λ�û�pidֵ
    const static fp32 Y_MOTION_motor_position_pid[3] = {Y_MOTION_M3505_MOTOR_POSITION_PID_KP,Y_MOTION_M3505_MOTOR_POSITION_PID_KI, Y_MOTION_M3505_MOTOR_POSITION_PID_KD};
    //����һ���˲�����
    const static fp32 chassis_y_order_filter[1] = {MOTION_ACCEL_Y_NUM};

    //���̿���״̬Ϊֹͣ
    XYZ_MOTION_move_init->Y_MOTION_mode = Y_MOTION_STOP;	
/**** ң��������ָ���ȡ ****/																											
    
    XYZ_MOTION_move_init->Y_MOTION_System_RC = get_remote_control_point();

/**** �������ָ���ȡ ****/
		
    XYZ_MOTION_move_init->Y_MOTION_Left_motor.Y_MOTION_motor_measure = get_Y_LEFT_MOTOR_Measure_Point();
		XYZ_MOTION_move_init->Y_MOTION_Right_motor.Y_MOTION_motor_measure = get_Y_RIGHT_MOTOR_Measure_Point();

/**** ��ʼ��PID �˶� ****/
	
		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_Left_motor_Speed_pid, PID_POSITION, Y_MOTION_motor_speed_pid,
							Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_IOUT);
		
		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_Left_motor_Position_pid, PID_POSITION, Y_MOTION_motor_position_pid,
							Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_IOUT);
							
		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_Right_motor_Speed_pid, PID_POSITION, Y_MOTION_motor_speed_pid,
							Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_SPEED_PID_MAX_IOUT);

		PID_Init(&XYZ_MOTION_move_init->Y_MOTION_Left_motor_Position_pid, PID_POSITION, Y_MOTION_motor_position_pid,
							Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_OUT, Y_MOTION_M3505_MOTOR_POSITION_PID_MAX_IOUT);

   
		//��һ���˲�����б����������
		//��ʼ����ſ���ʹ��һ�׵�ͨ�˲�
    first_order_filter_init(&XYZ_MOTION_move_init->chassis_cmd_slow_set_vy, Y_MOTION_CONTROL_TIME, chassis_y_order_filter);

    //�޷���Y������˶��ٶȡ�MAX and MIN��
    XYZ_MOTION_move_init->vy_max_speed = Y_MOTION_MAX_SPEED;
    XYZ_MOTION_move_init->vy_min_speed = -Y_MOTION_MAX_SPEED;

    //����һ������
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
	/*********** ����chassis_move->motor_chassis[i].speed��accel ***********/
        //���µ��������ʵ���ٶȣ����ٶ����ٶȵ�PID΢��
				XYZ_MOTION_move_update->Y_MOTION_Left_motor.speed = XYZ_MOTION_move_update->Y_MOTION_Left_motor.Y_MOTION_motor_measure->speed_rpm;
				XYZ_MOTION_move_update->Y_MOTION_Right_motor.speed = XYZ_MOTION_move_update->Y_MOTION_Right_motor.Y_MOTION_motor_measure->speed_rpm;
																								
        XYZ_MOTION_move_update->Y_MOTION_Left_motor.accel = XYZ_MOTION_move_update->Y_MOTION_Left_motor_Speed_pid.Dbuf[0]*Y_MOTION_CONTROL_FREQUENCE;
		    XYZ_MOTION_move_update->Y_MOTION_Right_motor.accel = XYZ_MOTION_move_update->Y_MOTION_Right_motor_Speed_pid.Dbuf[0]*Y_MOTION_CONTROL_FREQUENCE;

    

	/********** ���ݸ��µ�chassis_move->motor_chassis[i].speed������Vx��Vy��Wz ******************************/
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

	//��ң����ͨ�����ֵvy_set(m/s) ת���� �������ת��rpm
    XYZ_MOTION_rc_to_motor_speed(XYZ_MOTION_move->vy_set, &motor_speed);

    if (XYZ_MOTION_move->Y_MOTION_mode == Y_MOTION_STOP)
    {
        //��ֵ����ֵ
       XYZ_MOTION_move->Y_MOTION_Left_motor.give_current = 0.0f;
			 XYZ_MOTION_move->Y_MOTION_Right_motor.give_current = 0.0f;

        //raw����ֱ�ӷ���
        return;
    }

    //�������ӿ�������ٶȣ�������������ٶ�
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

    //����pid
    PID_Calc(&XYZ_MOTION_move->Y_MOTION_Left_motor_Speed_pid, 
		XYZ_MOTION_move->Y_MOTION_Left_motor.speed, XYZ_MOTION_move->Y_MOTION_Left_motor.speed_set);
		PID_Calc(&XYZ_MOTION_move->Y_MOTION_Right_motor_Speed_pid, 
		XYZ_MOTION_move->Y_MOTION_Right_motor.speed, XYZ_MOTION_move->Y_MOTION_Right_motor.speed_set);

    //��ֵ����ֵ
		XYZ_MOTION_move->Y_MOTION_Left_motor.give_current= (int16_t)(XYZ_MOTION_move->Y_MOTION_Left_motor_Speed_pid.out);
		XYZ_MOTION_move->Y_MOTION_Right_motor.give_current= (int16_t)(XYZ_MOTION_move->Y_MOTION_Right_motor_Speed_pid.out);

}

static void XYZ_MOTION_rc_to_motor_speed(const fp32 vy_set, fp32 *Motor_Speed)
{
 //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
    *Motor_Speed = 9.6f * (1/SychronicWheelRadius) * vy_set;   // V(m/s)=2 * PI * R(m) * N(Round Per Second)
 
}


void XYZ_MOTION_rc_filter(fp32 *vy_set, XYZ_MOTION_System_t *XYZ_MOTION_move)
{
    if (XYZ_MOTION_move == NULL || vy_set == NULL)
    {
        return;
    }
    //ң����ԭʼͨ��ֵ
    int16_t vy_channel;
    fp32 vy_set_channel;
		
//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
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

//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
		// In:vx_set_channel   Out:chassis_move->chassis_cmd_slow_set_vx
		// In:vy_set_channel   Out:chassis_move->chassis_cmd_slow_set_vy
    first_order_filter_cali(&XYZ_MOTION_move->chassis_cmd_slow_set_vy, vy_set_channel);

//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
		//���˻��������ٶ�Ϊ0
    if (vy_set_channel < Y_MOTION_RC_DEADLINE * Y_MOTION_RC_SEN && vy_set_channel > -Y_MOTION_RC_DEADLINE * Y_MOTION_RC_SEN)
    {
        XYZ_MOTION_move->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		
    *vy_set = XYZ_MOTION_move->chassis_cmd_slow_set_vy.out;
}



