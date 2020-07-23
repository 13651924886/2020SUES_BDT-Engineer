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
    //ң����ԭʼͨ��ֵ
    int16_t angular_velocity_channel;
    fp32 angular_velocity_set_channel;
		
//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
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

//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
		// In:vx_set_channel   Out:chassis_move->chassis_cmd_slow_set_vx
		// In:vy_set_channel   Out:chassis_move->chassis_cmd_slow_set_vy
    first_order_filter_cali(&GRIPPER_move->Gripper_cmd_slow_set_Angular_Velocity, angular_velocity_set_channel);

//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
		//���˻��������ٶ�Ϊ0
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

    //�ٶȻ�pidֵ
    const static fp32 GRIPPER_motor_angular_speed_pid[3] = {GRIPPER_M3505_MOTOR_SPEED_PID_KP, GRIPPER_M3505_MOTOR_SPEED_PID_KI, GRIPPER_M3505_MOTOR_SPEED_PID_KD};
    //λ�û�pidֵ
    const static fp32 GRIPPER_motor_position_pid[3] = {GRIPPER_M3505_MOTOR_POSITION_PID_KP,GRIPPER_M3505_MOTOR_POSITION_PID_KI, GRIPPER_M3505_MOTOR_POSITION_PID_KD};
    //����һ���˲�����
    const static fp32 Gripper_order_filter[1] = {MOTION_ACCEL_Y_NUM};

    //���̿���״̬Ϊֹͣ
    GRIPPER_move_init->GRIPPER_mode = GRIPPER_STOP;	
/**** ң��������ָ���ȡ ****/																											
    
    GRIPPER_move_init->GRIPPER_System_RC = get_remote_control_point();

/**** �������ָ���ȡ ****/
		
    GRIPPER_move_init->GRIPPER_1_motor.gripper_motor_measure = get_GRIPPER_1_MOTOR_Measure_Point();
		GRIPPER_move_init->GRIPPER_2_motor.gripper_motor_measure = get_GRIPPER_2_MOTOR_Measure_Point();
		
/**** ���OFFSET��ȡ,���ڼ���Relative Angle****/
		
		GRIPPER_move_init->GRIPPER_1_motor.offset_ecd = GRIPPER_MOTOR1_RELATIVE_ANGLE_OFFSET;
		GRIPPER_move_init->GRIPPER_2_motor.offset_ecd = GRIPPER_MOTOR2_RELATIVE_ANGLE_OFFSET;

/**** ��ʼ��PID �˶� ****/
	
		PID_Init(&GRIPPER_move_init->GRIPPER_1_motor_Angular_Speed_pid, PID_POSITION, GRIPPER_motor_angular_speed_pid,
							GRIPPER_M3505_MOTOR_SPEED_PID_MAX_OUT, GRIPPER_M3505_MOTOR_SPEED_PID_MAX_IOUT);
		
		PID_Init(&GRIPPER_move_init->GRIPPER_1_motor_Position_pid, PID_POSITION, GRIPPER_motor_position_pid,
							GRIPPER_M3505_MOTOR_POSITION_PID_MAX_OUT, GRIPPER_M3505_MOTOR_POSITION_PID_MAX_IOUT);
							
		PID_Init(&GRIPPER_move_init->GRIPPER_2_motor_Angular_Speed_pid, PID_POSITION, GRIPPER_motor_angular_speed_pid,
							GRIPPER_M3505_MOTOR_SPEED_PID_MAX_OUT, GRIPPER_M3505_MOTOR_SPEED_PID_MAX_IOUT);

		PID_Init(&GRIPPER_move_init->GRIPPER_1_motor_Position_pid, PID_POSITION, GRIPPER_motor_position_pid,
							GRIPPER_M3505_MOTOR_POSITION_PID_MAX_OUT, GRIPPER_M3505_MOTOR_POSITION_PID_MAX_IOUT);

   
		//��һ���˲�����б����������
		//��ʼ����ſ���ʹ��һ�׵�ͨ�˲�
    first_order_filter_init(&GRIPPER_move_init->Gripper_cmd_slow_set_Angular_Velocity, GRIPPER_CONTROL_TIME, Gripper_order_filter);

    //�޷���Y������˶��ٶȡ�MAX and MIN��
    GRIPPER_move_init->Angular_Velocity_max_speed = GRIPPER_MAX_SPEED;
    GRIPPER_move_init->Angular_Velocity_min_speed = -GRIPPER_MAX_SPEED;
		
		//Ŀǰ��צ����ԽǶ�λ����Motor1����ԽǶ�������
		GRIPPER_move_init->Position_Max = GRIPPER_MAX_POSITION;
		GRIPPER_move_init->Position_Min = GRIPPER_MIN_POSITION;

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
	/*********** ����chassis_move->motor_chassis[i].speed��accel ***********/
        //���µ��������ʵ���ٶȣ����ٶ����ٶȵ�PID΢��
				GRIPPER_move_update->GRIPPER_1_motor.speed = GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->speed_rpm;
				GRIPPER_move_update->GRIPPER_2_motor.speed = GRIPPER_move_update->GRIPPER_2_motor.gripper_motor_measure->speed_rpm;
																								
        GRIPPER_move_update->GRIPPER_1_motor.accel = GRIPPER_move_update->GRIPPER_1_motor_Angular_Speed_pid.Dbuf[0]*GRIPPER_CONTROL_FREQUENCE;
		    GRIPPER_move_update->GRIPPER_2_motor.accel = GRIPPER_move_update->GRIPPER_2_motor_Angular_Speed_pid.Dbuf[0]*GRIPPER_CONTROL_FREQUENCE;
				
				GRIPPER_move_update->GRIPPER_1_motor.relative_angle = motor_ecd_to_angle_change(GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->ecd,GRIPPER_move_update->GRIPPER_1_motor.offset_ecd);
				GRIPPER_move_update->GRIPPER_2_motor.relative_angle = motor_ecd_to_angle_change(GRIPPER_move_update->GRIPPER_2_motor.gripper_motor_measure->ecd,GRIPPER_move_update->GRIPPER_2_motor.offset_ecd);

				//Ŀǰץ��λ����Motor1����ԽǶ�������
				GRIPPER_move_update->Position = motor_ecd_to_angle_change(GRIPPER_move_update->GRIPPER_1_motor.gripper_motor_measure->ecd,GRIPPER_move_update->GRIPPER_1_motor.offset_ecd);
		
	/********** ���ݸ��µ�chassis_move->motor_chassis[i].speed������Vx��Vy��Wz ******************************/
				//Ŀǰץ�ֽ��ٶ���Motor1�Ľ��ٶ�������
				GRIPPER_move_update->Angular_Velocity = GRIPPER_move_update->GRIPPER_1_motor.speed; //���ٶ�ʵ���ϲ�����RPMת�٣�֮���ٸ�
				//Ŀǰץ��λ����Motor1����ԽǶ�������
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

	//��ң����ͨ�����ֵAngular_Velocity_set(��/s) ת���� �������ת��rpm
    GRIPPER_AngularVelocity_rc_to_motor_speed(GRIPPER_move->Angular_Velocity_set, &motor_speed);

    if (GRIPPER_move->GRIPPER_mode == GRIPPER_STOP)
    {
        //��ֵ����ֵ
       GRIPPER_move->GRIPPER_1_motor.give_current = 0.0f;
			 GRIPPER_move->GRIPPER_2_motor.give_current = 0.0f;

        //raw����ֱ�ӷ���
        return;
    }
//		}
		
		GRIPPER_move->GRIPPER_1_motor.speed_set = -motor_speed;
		GRIPPER_move->GRIPPER_2_motor.speed_set = motor_speed;

    //����pid
    PID_Calc(&GRIPPER_move->GRIPPER_1_motor_Angular_Speed_pid, 
		GRIPPER_move->GRIPPER_1_motor.speed, GRIPPER_move->GRIPPER_1_motor.speed_set);
		PID_Calc(&GRIPPER_move->GRIPPER_2_motor_Angular_Speed_pid, 
		GRIPPER_move->GRIPPER_2_motor.speed, GRIPPER_move->GRIPPER_2_motor.speed_set);

    //��ֵ����ֵ
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
    if (relative_ecd > Half_ecd_range) ///�������ֵ��ֵHalf_ecd_range = 4096
    {
        relative_ecd -= ecd_range;		///�������ֵ���ֵecd_range = 8191
    }
    else if (relative_ecd < -Half_ecd_range)//-Half_ecd_range = -4096
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;//�������ֵת���ɽǶ�ֵ rad
}

static fp32 motor_ecd_to_position_change(GRIPPER_System_t *Gripper_ecd, uint16_t *po  );

