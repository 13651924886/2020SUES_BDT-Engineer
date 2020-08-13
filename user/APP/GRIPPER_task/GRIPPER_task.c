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
static void GRIPPER_mode_transit(GRIPPER_System_t *GRIPPER_move);
static void GRIPPER_feedback_update(GRIPPER_System_t *GRIPPER_move_update);
static void GRIPPER_set_mode(GRIPPER_System_t *GRIPPER_mode);
static void GRIPPER_set_contorl(GRIPPER_System_t *GRIPPER_move);
static void GRIPPER_control_loop(GRIPPER_System_t *GRIPPER_control_loop);
		
static void GPRIPPER_MOTOR1_PID_Task(GRIPPER_System_t *GRIPPER_move);
static void GPRIPPER_MOTOR2_PID_Task(GRIPPER_System_t *GRIPPER_move);



void GRIPPER_Setup(void)
{   
	GRIPPER_init(&GRIPPER_move);
}

void GRIPPER_task(void)
{
	GRIPPER_set_mode(&GRIPPER_move);
	GRIPPER_mode_transit(&GRIPPER_move);
	GRIPPER_feedback_update(&GRIPPER_move);
	GRIPPER_set_contorl(&GRIPPER_move);
	GRIPPER_control_loop(&GRIPPER_move);

#if MOTOR_ENABLE
	
	CAN2_CMD_GRIPPER(GRIPPER_move.GRIPPER_1_motor.give_current, GRIPPER_move.GRIPPER_2_motor.give_current,
                                0, 0);		
#endif
}


static void GRIPPER_init(GRIPPER_System_t *GRIPPER_move_init)
{
    if (GRIPPER_move_init == NULL)
    {
        return;
    }

    //�ٶȻ�pidֵ
    const static fp32 GRIPPER_motor1_angular_speed_pid[3] = {GRIPPER_M3505_MOTOR1_SPEED_PID_KP, GRIPPER_M3505_MOTOR1_SPEED_PID_KI, GRIPPER_M3505_MOTOR1_SPEED_PID_KD};
    const static fp32 GRIPPER_motor2_angular_speed_pid[3] = {GRIPPER_M3505_MOTOR1_SPEED_PID_KP, GRIPPER_M3505_MOTOR1_SPEED_PID_KI, GRIPPER_M3505_MOTOR1_SPEED_PID_KD};
    //λ�û�pidֵ
    const static fp32 GRIPPER_motor1_position_pid[3] = {GRIPPER_M3505_MOTOR1_POSITION_PID_KP,GRIPPER_M3505_MOTOR1_POSITION_PID_KI, GRIPPER_M3505_MOTOR1_POSITION_PID_KD};
    const static fp32 GRIPPER_motor2_position_pid[3] = {GRIPPER_M3505_MOTOR1_POSITION_PID_KP,GRIPPER_M3505_MOTOR1_POSITION_PID_KI, GRIPPER_M3505_MOTOR1_POSITION_PID_KD};
    //����һ���˲�����
    const static fp32 second_order_fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���̿���״̬Ϊֹͣ
    GRIPPER_move_init->GRIPPER_mode = GRIPPER_STOP;	
/**** ң��������ָ���ȡ ****/																											
    
    GRIPPER_move_init->GRIPPER_System_RC = get_remote_control_point();
		
		second_order_filter_init(&GRIPPER_move_init->GRIPPER_1_motor.second_order_filter, second_order_fliter_num);
		second_order_filter_init(&GRIPPER_move_init->GRIPPER_2_motor.second_order_filter, second_order_fliter_num);

/**** �������ָ���ȡ ****/
		
    GRIPPER_move_init->GRIPPER_1_motor.gripper_motor_measure = get_GRIPPER_1_MOTOR_Measure_Point();
		GRIPPER_move_init->GRIPPER_2_motor.gripper_motor_measure = get_GRIPPER_2_MOTOR_Measure_Point();
		
/**** ��ʼ��PID �˶� ****/
	
		PID_Init(&GRIPPER_move_init->GRIPPER_1_motor_Angular_Speed_pid, PID_POSITION, GRIPPER_motor1_angular_speed_pid,
							GRIPPER_M3505_MOTOR1_SPEED_PID_MAX_OUT, GRIPPER_M3505_MOTOR1_SPEED_PID_MAX_IOUT);
		
		PID_Init(&GRIPPER_move_init->GRIPPER_1_motor_Position_pid, PID_POSITION, GRIPPER_motor1_position_pid,
							GRIPPER_M3505_MOTOR1_POSITION_PID_MAX_OUT, GRIPPER_M3505_MOTOR1_POSITION_PID_MAX_IOUT);
							
		PID_Init(&GRIPPER_move_init->GRIPPER_2_motor_Angular_Speed_pid, PID_POSITION, GRIPPER_motor2_angular_speed_pid,
							GRIPPER_M3505_MOTOR2_SPEED_PID_MAX_OUT, GRIPPER_M3505_MOTOR2_SPEED_PID_MAX_IOUT);

		PID_Init(&GRIPPER_move_init->GRIPPER_2_motor_Position_pid, PID_POSITION, GRIPPER_motor2_position_pid,
							GRIPPER_M3505_MOTOR2_POSITION_PID_MAX_OUT, GRIPPER_M3505_MOTOR2_POSITION_PID_MAX_IOUT);

		
		//Ŀǰ��צ����ԽǶ�λ����Motor1����ԽǶ�������
		GRIPPER_move_init->Position_Max = GRIPPER_MAX_POSITION;
		GRIPPER_move_init->Position_Min = GRIPPER_MIN_POSITION;
		//Ȧ����ʼ��Ϊ0
		GRIPPER_move_init->GRIPPER_1_motor.round_count = 0;
		GRIPPER_move_init->GRIPPER_2_motor.round_count = 0;
    //����һ������
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
			GRIPPER_move->GRIPPER_mode = GRIPPER_ENGAGE;
		}
		if (switch_is_down(rc_ctrl.rc.s[0]))
		{
			GRIPPER_move->GRIPPER_mode = GRIPPER_HOME;
		}
	 if (switch_is_mid(rc_ctrl.rc.s[0]))
		{
			GRIPPER_move->GRIPPER_mode = GRIPPER_STOP;
		}
}
static void GRIPPER_mode_transit(GRIPPER_System_t *GRIPPER_move)
{
		if (GRIPPER_move == NULL)
    {
        return;
    }
		
		if (GRIPPER_move->last_GRIPPER_mode == GRIPPER_move->GRIPPER_mode)
    {
        return;
    }
//		if ((XYZ_MOTION_move->last_XYZ_mode != HOME) && XYZ_MOTION_move->XYZ_mode == ENGAGE)
//    {
//        XYZ_MOTION_move->Y_MOTION_System.Position_set = 0.0f;
//    }
//		
		GRIPPER_move->last_GRIPPER_mode = GRIPPER_move->GRIPPER_mode;
}
static void GRIPPER_feedback_update(GRIPPER_System_t *GRIPPER_move_update)
{
	   if (GRIPPER_move_update == NULL)
    {
        return;
    }
		static fp32 init_ecd1 = 0.0f;
		static fp32 init_ecd2 = 0.0f;
		
		if(init_ecd1 == 0)init_ecd1 	= GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->ecd;
		if(init_ecd2 == 0)init_ecd2 	= GRIPPER_move_update->GRIPPER_2_motor.gripper_motor_measure->ecd;
	/*********** ����chassis_move->motor_chassis[i].speed��accel ***********/
		//���׵�ͨ�˲�M3508��ת�٣��Ӷ��õ����ٶ�speed
		second_order_filter_calc(&GRIPPER_move_update->GRIPPER_1_motor.second_order_filter,
														 GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->speed_rpm * M3508_Motor_RPM_TO_SPEED);
		second_order_filter_calc(&GRIPPER_move_update->GRIPPER_2_motor.second_order_filter,
														 GRIPPER_move_update->GRIPPER_2_motor.gripper_motor_measure->speed_rpm * M3508_Motor_RPM_TO_SPEED);
		
		GRIPPER_move_update->GRIPPER_1_motor.speed = GRIPPER_move_update->GRIPPER_1_motor.second_order_filter.output;
		GRIPPER_move_update->GRIPPER_2_motor.speed = GRIPPER_move_update->GRIPPER_2_motor.second_order_filter.output;
																								
    GRIPPER_move_update->GRIPPER_1_motor.accel = GRIPPER_move_update->GRIPPER_1_motor_Angular_Speed_pid.Dbuf[0]*GRIPPER_CONTROL_FREQUENCE;
		GRIPPER_move_update->GRIPPER_2_motor.accel = GRIPPER_move_update->GRIPPER_2_motor_Angular_Speed_pid.Dbuf[0]*GRIPPER_CONTROL_FREQUENCE;
				
		GRIPPER_move_update->GRIPPER_1_motor.Position = Motor_RoundCount_Position_Calc(GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->ecd,
																																			GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->last_ecd,
																																			&GRIPPER_move_update->GRIPPER_1_motor.round_count,
																																			init_ecd1);
		GRIPPER_move_update->GRIPPER_2_motor.Position = Motor_RoundCount_Position_Calc(GRIPPER_move_update->GRIPPER_2_motor.gripper_motor_measure->ecd,
																																			GRIPPER_move_update->GRIPPER_2_motor.gripper_motor_measure->last_ecd,
																																			&GRIPPER_move_update->GRIPPER_2_motor.round_count,
																																			init_ecd2);																																	
}

static void GRIPPER_set_contorl(GRIPPER_System_t *GRIPPER_move)
{
	
	if (GRIPPER_move == NULL)
  {
        return;
  }
	static uint16_t last_turn_keyboard = 0;
  static uint8_t 	mode_turn_flag = 0;
	static int64_t gripper_motor1_position_set = GPRIPPER_MOTOR1_INIT_POSITION;
	static int64_t gripper_motor2_position_set = GPRIPPER_MOTOR2_INIT_POSITION;

			
	
	if (GRIPPER_move->GRIPPER_mode ==	GRIPPER_STOP)
	{
		GRIPPER_move->GRIPPER_1_motor.give_current = 0;
		GRIPPER_move->GRIPPER_2_motor.give_current = 0;
	}
	if (GRIPPER_move->GRIPPER_mode ==	GRIPPER_HOME)
	{
		GRIPPER_move->GRIPPER_1_motor.Position_set = GPRIPPER_MOTOR1_INIT_POSITION;
		GRIPPER_move->GRIPPER_2_motor.Position_set = GPRIPPER_MOTOR2_INIT_POSITION;
	}	
	if (GRIPPER_move->GRIPPER_mode ==	GRIPPER_ENGAGE)
	{
		//�̰�һ�μ��
		if(	(GRIPPER_move->GRIPPER_System_RC->key.v & GPRIPPER_KEY) && !(last_turn_keyboard & GPRIPPER_KEY) )
		{
				 if (mode_turn_flag == 0)
				 {
						mode_turn_flag = 1;		
				 }
		}
		
	  last_turn_keyboard = GRIPPER_move->GRIPPER_System_RC->key.v ;
		
	  if (mode_turn_flag)
	  {
				static u8 i = 0;
				if(GRIPPER_move->GRIPPER_System_RC->key.v & GPRIPPER_KEY && i == 0)	
				{
					if( gripper_motor1_position_set == GPRIPPER_MOTOR1_INIT_POSITION)
					{
						gripper_motor1_position_set +=  GRIPPER_ADD_POSITION;
						gripper_motor2_position_set += GRIPPER_ADD_POSITION;
					}
					if( gripper_motor1_position_set != GPRIPPER_MOTOR1_INIT_POSITION)
					{
						gripper_motor1_position_set -=  GRIPPER_ADD_POSITION;
						gripper_motor2_position_set -= GRIPPER_ADD_POSITION;
					}
					i++;
				}	
				i = 0;
				mode_turn_flag = 0;
		}
	}	
	
	GRIPPER_move->GRIPPER_1_motor.Position_set = gripper_motor1_position_set;
	GRIPPER_move->GRIPPER_2_motor.Position_set = gripper_motor2_position_set;
}

static void GRIPPER_control_loop(GRIPPER_System_t *GRIPPER_move)
{
	//��ң����ͨ�����ֵAngular_Velocity_set(��/s) ת���� �������ת��rpm
    if (GRIPPER_move->GRIPPER_mode == GRIPPER_STOP)
    {
        //��ֵ����ֵ
       GRIPPER_move->GRIPPER_1_motor.give_current = 0.0f;
			 GRIPPER_move->GRIPPER_2_motor.give_current = 0.0f;

        //raw����ֱ�ӷ���
        return;
    }
		GPRIPPER_MOTOR1_PID_Task(GRIPPER_move);
		GPRIPPER_MOTOR2_PID_Task(GRIPPER_move);

}

static void GPRIPPER_MOTOR1_PID_Task(GRIPPER_System_t *GRIPPER_move)
{
			//�ǶȻ��⻷	λ��ʽ
		GRIPPER_move->GRIPPER_1_motor.speed_set = PID_Calc(&GRIPPER_move->GRIPPER_1_motor_Angular_Speed_pid,
																	GRIPPER_move->GRIPPER_1_motor.Position, GRIPPER_move->GRIPPER_1_motor.Position_set);
		//���ٶȻ��ڻ� λ��ʽ
		GRIPPER_move->GRIPPER_1_motor.current_set = PID_Calc(&GRIPPER_move->GRIPPER_1_motor_Position_pid,
																	GRIPPER_move->GRIPPER_1_motor.speed,GRIPPER_move->GRIPPER_1_motor.speed_set);
    //����ֵ��ֵ
    GRIPPER_move->GRIPPER_1_motor.give_current = (int16_t)(GRIPPER_move->GRIPPER_1_motor.current_set);
}

static void GPRIPPER_MOTOR2_PID_Task(GRIPPER_System_t *GRIPPER_move)
{
			//�ǶȻ��⻷	λ��ʽ
		GRIPPER_move->GRIPPER_2_motor.speed_set = PID_Calc(&GRIPPER_move->GRIPPER_2_motor_Angular_Speed_pid,
																	GRIPPER_move->GRIPPER_2_motor.Position, GRIPPER_move->GRIPPER_2_motor.Position_set);
		//���ٶȻ��ڻ� λ��ʽ
		GRIPPER_move->GRIPPER_2_motor.current_set = PID_Calc(&GRIPPER_move->GRIPPER_2_motor_Position_pid,
																	GRIPPER_move->GRIPPER_2_motor.speed,GRIPPER_move->GRIPPER_2_motor.speed_set);
    //����ֵ��ֵ
    GRIPPER_move->GRIPPER_2_motor.give_current = (int16_t)(GRIPPER_move->GRIPPER_2_motor.current_set);
}
