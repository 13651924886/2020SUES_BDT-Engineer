#include "CATCH_task.h"

extern RC_ctrl_t rc_ctrl;

CATCH_System_t CATCH_move;

static void CATCH_init(CATCH_System_t *CATCH_move_init);
static void CATCH_set_mode(CATCH_System_t *CATCH_move);
static void	CATCH_mode_transit(CATCH_System_t *CATCH_move);
static void CATCH_feedback_update(CATCH_System_t *CATCH_move_init);
static void CATCH_contorl(CATCH_System_t *CATCH_move);

void CATCH_Setup(void)
{
		CATCH_init(&CATCH_move);
}

void CATCH_task(void)
{
	CATCH_set_mode(&CATCH_move);
	CATCH_mode_transit(&CATCH_move);
	CATCH_feedback_update(&CATCH_move);
	CATCH_contorl(&CATCH_move);
}
static void CATCH_init(CATCH_System_t *CATCH_move_init)
{
    if (CATCH_move_init == NULL)
    {
        return;
    }
    //底盘开机状态为停止
    CATCH_move_init->CATCH_Status = CATCH_STOP;	
/**** 遥控器数据指针获取 ****/																											
    
    CATCH_move_init->CATCH_System_RC = get_remote_control_point();
		
		CATCH_move_init->ControlFun = GPIO_CMD_Cylinder;
    //更新一下数据
    CATCH_feedback_update(CATCH_move_init);
}

static void CATCH_feedback_update(CATCH_System_t *CATCH_move_init)
{
	    //鼠标按键
    CATCH_move_init->last_press_l = CATCH_move_init->press_l;
    CATCH_move_init->last_press_r = CATCH_move_init->press_r;
    CATCH_move_init->press_l = CATCH_move_init->CATCH_System_RC->mouse.press_l;
    CATCH_move_init->press_r = CATCH_move_init->CATCH_System_RC->mouse.press_r;
    //长按计时
    if (CATCH_move_init->press_l)
    {
        if (CATCH_move_init->press_l_time < PRESS_LONG_TIME)
        {
            CATCH_move_init->press_l_time++;
        }
    }
    else
    {
        CATCH_move_init->press_l_time = 0;
    }

    if (CATCH_move_init->press_r)
    {
        if (CATCH_move_init->press_r_time < PRESS_LONG_TIME)
        {
           CATCH_move_init->press_r_time++;
        }
    }
    else
    {
        CATCH_move_init->press_r_time = 0;
    }
		
		CATCH_move_init->CATCH_Cylinder_GPIO = GPIO_ReadOutputDataBit(Catch_Cylinder_PORT,Catch_Cylinder_PIN);
		
}
static void CATCH_set_mode(CATCH_System_t *CATCH_move)
{
	 if (CATCH_move == NULL)
    {
        return;
    }
	 if (switch_is_up(rc_ctrl.rc.s[0]))
		{
			CATCH_move->CATCH_Status = CATCH_ENGAGE;
		}
	 if (switch_is_mid(rc_ctrl.rc.s[0]))
		{
			CATCH_move->CATCH_Status = CATCH_STOP;
		}
	 if(switch_is_down(rc_ctrl.rc.s[0]))
		{
			CATCH_move->CATCH_Status = CATCH_STOP;
		}
}
static void	CATCH_mode_transit(CATCH_System_t *CATCH_move)
{
	
	 if (CATCH_move == NULL)
    {
        return;
    }
		if (CATCH_move->last_CATCH_Status == CATCH_move->CATCH_Status)
    {
        return;
    }
//		if ((XYZ_MOTION_move->last_XYZ_mode != HOME) && XYZ_MOTION_move->XYZ_mode == ENGAGE)
//    {
//        XYZ_MOTION_move->Y_MOTION_System.Position_set = 0.0f;
//    }
//		
		CATCH_move->last_CATCH_Status = CATCH_move->CATCH_Status;
}

static void CATCH_contorl(CATCH_System_t *CATCH_move)
{
	if (CATCH_move == NULL)
  {
        return;
  }
		
	if (CATCH_move->CATCH_Status ==	CATCH_ENGAGE)
	{
		CATCH_move->ControlFun(Catch_Cylinder,OPEN);
	}
	if (CATCH_move->CATCH_Status ==	CATCH_STOP)
	{
		CATCH_move->ControlFun(Catch_Cylinder,CLOSE);
	}
}
