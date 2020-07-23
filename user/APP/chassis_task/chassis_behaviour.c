/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

//底盘行为状态机变量初始化
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_NO_MOVE;

//chassis_set_mode中用到
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

		/**************** 通过遥控器设置底盘行为状态机模式  ****************/
		
		//初始状态下为CHASSIS_ZERO_FORCE 为了在遥控器没开的情况下保证安全
		
		//检测遥控器拨扭是否朝下 -》 激活底盘不随动模式
    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
		//检测遥控器拨扭是否朝中或朝上 -》 激活底盘不移动模式 用于控制上层机构抓取
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
		else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
		
/**************** 根据底盘行为状态机选择底盘控制状态机  ****************/
    
    if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不移动，则设置底盘状态机为 底盘不跟随角度（期望全部为0） 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不跟随角度，则设置底盘状态机为 底盘不跟随角度 状态机。
    }

}

//chassis_set_contorl中用到
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
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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

/////////////分割线 以下程序为未来开发的拓展程序///////

//static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
//{
//    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
//    {
//        return;
//    }
//		
///******1.遥控器的数据处理成底盘的vx_set，vy_set******************/
//    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);


///******2.判断是否摇摆   angle_set********************************************/
//		
//    //摇摆角度是利用sin函数生成，swing_time 是sin函数的输入值
//    static fp32 swing_time = 0.0f;
//    //swing_time 是计算出来的角度
//    static fp32 swing_angle = 0.0f;
//    //max_angle 是sin函数的幅值
//    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
//    //add_time 是摇摆角度改变的快慢，最大越快
//    static fp32 const add_time = PI / 250.0f;
//    //使能摇摆标志位
//    static uint8_t swing_flag = 0;

//    //计算遥控器的原始输入信号

//    //判断是否要摇摆
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

//    //判断键盘输入是不是在控制底盘运动，底盘在运动减小摇摆角度
//    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
//        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
//    {
//        max_angle = SWING_MOVE_ANGLE;
//    }
//    else
//    {
//        max_angle = SWING_NO_MOVE_ANGLE;
//    }
//    //sin函数生成控制角度
//    if (swing_flag)
//    {
//        swing_angle = max_angle * arm_sin_f32(swing_time);
//        swing_time += add_time;
//    }
//    else
//    {
//        swing_angle = 0.0f;
//    }
//    //sin函数不超过2pi
//    if (swing_time > 2 * PI)
//    {
//        swing_time -= 2 * PI;
//    }

//    *angle_set = swing_angle;
//}
