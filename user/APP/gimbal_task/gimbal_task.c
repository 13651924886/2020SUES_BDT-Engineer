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
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  * @author         RM
  * @param[in]      �����ң����ֵ
  * @param[in]      ��������������ң����ֵ
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
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
	
	/*** YAW,PITCH��PID���� ****/
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		
/**** �������ָ���ȡ ****/
			//motor_yaw�ṹ����������Yaw��̨�����ʵʱ����
    Gimbal_System->gimbal_yaw_motor.gimbal_motor_measure = get_YAW_MOTOR_Measure_Point();		
			//motor_pit�ṹ����������Pitch��̨�����ʵʱ����
		Gimbal_System->gimbal_pitch_motor.gimbal_motor_measure = get_PITCH_MOTOR_Measure_Point();
		
/**** ң��������ָ���ȡ ****/
		 //rc_ctrl�ṹ�壬���ң����ʵʱͨ��ֵ
    Gimbal_System->gimbal_rc_ctrl = get_remote_control_point();	
		
/**** ��ʼ�����ģʽ ****/	
    Gimbal_System->gimbal_yaw_motor.gimbal_motor_mode = Gimbal_System->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_STOP;
    Gimbal_System->gimbal_pitch_motor.gimbal_motor_mode = Gimbal_System->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_STOP;
		
/**** ��ʼ��YAW��Pitch��PID ���������PID���ֵ������������ ****/		
		
		//��ʼ��yaw�����ԽǶȻ�PID
    GIMBAL_PID_Init(&Gimbal_System->gimbal_yaw_motor.gimbal_motor_relative_angle_pid,\
		YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT,\
		YAW_ENCODE_RELATIVE_PID_KP,YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
		//��ʼ��yaw������ٶȻ�PID
	  PID_Init(&Gimbal_System->gimbal_yaw_motor.gimbal_motor_speed_pid, PID_POSITION, Yaw_speed_pid,\
		YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    
		//��ʼ��pitch�����ԽǶȻ�PID
    GIMBAL_PID_Init(&Gimbal_System->gimbal_pitch_motor.gimbal_motor_relative_angle_pid,\
		PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT,\
		PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
		//��ʼ��pitch������ٶȻ�PID
    PID_Init(&Gimbal_System->gimbal_pitch_motor.gimbal_motor_speed_pid, PID_POSITION, Pitch_speed_pid,\
		PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);


    //�����̨PID�����������������ֵ��ȫ�����㣬��ֹ��������
    gimbal_total_pid_clear(Gimbal_System);
		
/**** ����ʵ��װ����������ƫ��offset�������С��λ ****/
		//������̨relative angle��offset,����ʵ��װ����������
		Gimbal_System->gimbal_yaw_motor.offset_ecd 	 = GIMBAL_YAW_RELATIVE_ANGLE_OFFSET;    
		Gimbal_System->gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_RELATIVE_ANGLE_OFFSET;	
		
		//������̨relative angle���ֵ����Сֵ ����е��λ���ֵ
		Gimbal_System->gimbal_yaw_motor.max_relative_angle = GIMBAL_YAW_MAX_RELATIVE_ANGLE;
		Gimbal_System->gimbal_yaw_motor.min_relative_angle = GIMBAL_YAW_MIN_RELATIVE_ANGLE;
		Gimbal_System->gimbal_pitch_motor.max_relative_angle = GIMBAL_PITCH_MAX_RELATIVE_ANGLE;
		Gimbal_System->gimbal_pitch_motor.min_relative_angle = GIMBAL_PITCH_MIN_RELATIVE_ANGLE;
/**** ����yaw��pitch��absolute_angle���ԽǶȡ�relative_angle��ԽǶȡ�motor_gyro���ٶ� ****/		
    GImbal_Feedback_Update(Gimbal_System);
		
		//����YAW��PITCH����Ϊ���ڵķ���
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
		
//////////////2006��� 

		static fp32 speed_filter[2][3] = {{ 0.0f,0.0f,0.0f },{ 0.0f,0.0f,0.0f }};

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
		
	  //���׵�ͨ�˲�	����
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


    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
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
		
		    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
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


    //���������Ƕ�
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
		
		fp32 add_yaw_angle = 0.0f;		//yaw��Ƕ���������
    fp32 add_pitch_angle = 0.0f;	//pitch��Ƕ���������
				
		
    gimbal_mode_control_set(&add_yaw_angle, &add_pitch_angle, Gimbal_System);
		
/******************�ڶ��� ������̨���״̬����ģʽ����yaw��pitch�Ƕ�����relative_angle_set/absolute_angle_set�޷� *******************************************************/
	/****************** yaw����Ƕ�����relative_angle_set/absolute_angle_set�޷� ***************************************/
			if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_STOP)
			{
					//rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
					gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
			}
			else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
			{
					//encondeģʽ�£��������Ƕȿ���
					GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, &add_yaw_angle);
			}
			
	/****************** pitch����Ƕ�����relative_angle_set/absolute_angle_set�޷� ***************************************/
			if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_STOP)
			{
					//rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
					gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
			}
			else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
			{
					//encondeģʽ�£��������Ƕȿ���
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
		
/********** ң�������ݴ�������+������x��y��ֵһ��ֵ��rc_add_yaw,rc_add_pit *******************************************************/
    //��gimbal_control->gimbal_rc_ctrl->rc.ch[YawChannel]���������� ��ֵ��yaw_channel		��static��
		//��gimbal_control->gimbal_rc_ctrl->rc.ch[PitchChannel]��������,��ֵ��pitch_channel ��static��
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);

		//yaw������ = k1����yaw_channel + k2�������x����ֵ
		//pitch������ = k1����pitch_channel + k2�������y����ֵ
    rc_add_yaw = yaw_channel * Yaw_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen;
    rc_add_pit = pitch_channel * Pitch_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * Pitch_Mouse_Sen;
		
/*********** ������̨��Ϊ״̬��gimbal_behaviour����ȡ��Ӧ���ƣ�����rc_add_yaw,rc_add_pit��ֵ��ȥ�õ�add_yaw_angle��add_pitch_angle	***************/
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
        gimbal_absolute_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);	//��ͷ����
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);	//return
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);			//rc_add_yaw��rc_add_pit��0
    }
    //��������������ֵ
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
//pid��������
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}


