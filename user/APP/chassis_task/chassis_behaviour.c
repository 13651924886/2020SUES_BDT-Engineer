/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      ��ɵ�����Ϊ����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"





static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

//������Ϊ״̬��������ʼ��
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_NO_MOVE;

//chassis_set_mode���õ�
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

		/**************** ͨ��ң�������õ�����Ϊ״̬��ģʽ  ****************/
		
		//��ʼ״̬��ΪCHASSIS_ZERO_FORCE Ϊ����ң����û��������±�֤��ȫ
		
		//���ң������Ť�Ƿ��� -�� ������̲��涯ģʽ
    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
		//���ң������Ť�Ƿ��л��� -�� ������̲��ƶ�ģʽ ���ڿ����ϲ����ץȡ
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
		else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
		
/**************** ���ݵ�����Ϊ״̬��ѡ����̿���״̬��  ****************/
    
    if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //����Ϊ�ǵ��̲��ƶ��������õ���״̬��Ϊ ���̲�����Ƕȣ�����ȫ��Ϊ0�� ״̬����
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //����Ϊ�ǵ��̲�����Ƕȣ������õ���״̬��Ϊ ���̲�����Ƕ� ״̬����
    }

}

//chassis_set_contorl���õ�
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }

    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/////////////�ָ��� ���³���Ϊδ����������չ����///////

//static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
//{
//    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
//    {
//        return;
//    }
//		
///******1.ң���������ݴ���ɵ��̵�vx_set��vy_set******************/
//    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);


///******2.�ж��Ƿ�ҡ��   angle_set********************************************/
//		
//    //ҡ�ڽǶ�������sin�������ɣ�swing_time ��sin����������ֵ
//    static fp32 swing_time = 0.0f;
//    //swing_time �Ǽ�������ĽǶ�
//    static fp32 swing_angle = 0.0f;
//    //max_angle ��sin�����ķ�ֵ
//    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
//    //add_time ��ҡ�ڽǶȸı�Ŀ��������Խ��
//    static fp32 const add_time = PI / 250.0f;
//    //ʹ��ҡ�ڱ�־λ
//    static uint8_t swing_flag = 0;

//    //����ң������ԭʼ�����ź�

//    //�ж��Ƿ�Ҫҡ��
//		if ((chassis_move_rc_to_vector->chassis_RC->rc.s[Swing])-2)
//    //if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
//    {
//        if (swing_flag == 0)
//        {
//            swing_flag = 1;
//            swing_time = 0.0f;
//        }
//    }
//    else
//    {
//        swing_flag = 0;
//    }

//    //�жϼ��������ǲ����ڿ��Ƶ����˶����������˶���Сҡ�ڽǶ�
//    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
//        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
//    {
//        max_angle = SWING_MOVE_ANGLE;
//    }
//    else
//    {
//        max_angle = SWING_NO_MOVE_ANGLE;
//    }
//    //sin�������ɿ��ƽǶ�
//    if (swing_flag)
//    {
//        swing_angle = max_angle * arm_sin_f32(swing_time);
//        swing_time += add_time;
//    }
//    else
//    {
//        swing_angle = 0.0f;
//    }
//    //sin����������2pi
//    if (swing_time > 2 * PI)
//    {
//        swing_time -= 2 * PI;
//    }

//    *angle_set = swing_angle;
//}
