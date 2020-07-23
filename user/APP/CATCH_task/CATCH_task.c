#include "CATCH_task.h"

extern RC_ctrl_t rc_ctrl;

CATCH_System_t CATCH_move;

static void CATCH_init(CATCH_System_t *CATCH_move_init);
static void CATCH_GPIO_init(void);
static void CATCH_set_mode(CATCH_System_t *CATCH_move);
static void CATCH_set_contorl(CATCH_System_t *CATCH_move);
//static void CATCH_feedback_update(CATCH_System_t *CATCH_move_update);
//static void CATCH_set_mode(CATCH_System_t *CATCH_mode);
//static void CATCH_set_contorl(CATCH_System_t *CATCH_move);
//static void CATCH_control_loop(CATCH_System_t *CATCH_control_loop);
//static void CATCH_AngularVelocity_rc_to_motor_speed(const fp32 angular_velocity_set, fp32 *Motor_Speed);

void CATCH_Setup(void)
{
		CATCH_init(&CATCH_move);
}

void CATCH_task(void)
{
	CATCH_set_mode(&CATCH_move);
	//CATCH_feedback_update(&CATCH_move);
	CATCH_set_contorl(&CATCH_move);
}
static void CATCH_init(CATCH_System_t *CATCH_move_init)
{
    if (CATCH_move_init == NULL)
    {
        return;
    }
		CATCH_GPIO_init();
    //底盘开机状态为停止
    CATCH_move_init->CATCH_Status = CATCH_OPEN;	
/**** 遥控器数据指针获取 ****/																											
    
    CATCH_move_init->CATCH_System_RC = get_remote_control_point();

    //更新一下数据
    //chassis_feedback_update(CATCH_move_init);
}
	
static void CATCH_GPIO_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
		
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);

}

static void CATCH_set_mode(CATCH_System_t *CATCH_move)
{
	 if (CATCH_move == NULL)
    {
        return;
    }
	 if (switch_is_up(rc_ctrl.rc.s[1]))
		{
			CATCH_move->CATCH_Status = CATCH_OPEN;
		}
	 if (switch_is_mid(rc_ctrl.rc.s[1]))
		{
			CATCH_move->CATCH_Status = CATCH_CLOSE;
		}
	 if(switch_is_down(rc_ctrl.rc.s[1]))
		{
			CATCH_move->CATCH_Status = CATCH_OPEN;
		}
}

static void CATCH_set_contorl(CATCH_System_t *CATCH_move)
{
	if (CATCH_move == NULL)
  {
        return;
  }
		
	if (CATCH_move->CATCH_Status ==	CATCH_OPEN)
	{
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);
	}
	if (CATCH_move->CATCH_Status ==	CATCH_CLOSE)
	{
		GPIO_SetBits(GPIOF, GPIO_Pin_10);

	}
}
