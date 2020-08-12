#include "Start_Task.h"

#include "Detect_Task.h"
#include "Chassis_Task.h"
#include "delay.h"
#include "User_Task.h"
#include "buzzer.h"
#include "XYZ_MOTION_task.h"
#include "GRIPPER_task.h"
#include "CATCH_task.h"
#include "CAN_Receive.h"
#include "AMMO_OUT_task.h"
#include "Revive_task.h"
#include "RescueHook_task.h"

#include "referee_usart_task.h"


//͵����ʱ�ӵ� ֮��Ҫд�ɶ���
#include "Remote_control.h"
#include "AMMO_OUT_task.h"
uint32_t test_dT_1000hz[3],test_rT[6];

extern XYZ_MOTION_System_t XYZ_MOTION_move;
static void Loop_1000Hz(void)	//1msִ��һ�Σ�ִ��ʱ���ԼΪ
{
	test_dT_1000hz[0] = test_dT_1000hz[1];
	test_rT[3] = test_dT_1000hz[1] = GetSysTime_us ();
	test_dT_1000hz[2] = (u32)(test_dT_1000hz[1] - test_dT_1000hz[0]) ;//test_dT_1000hz[2] �Ǹ�����ʱ��Ƭ����
//////////////////////////////////////////////////////////////////////	
			//3�Ų̛̉�˶�
			XYZ_MOTION_task();
//			GRIPPER_task();
//			CATCH_task();
//			ReviveCard_ReachOUT();
//			CAN_CMD_Upper(XYZ_MOTION_move.Y_MOTION_motor.give_current,-XYZ_MOTION_move.Y_MOTION_motor.give_current,
//                                GRIPPER_move.GRIPPER_1_motor.give_current, ReviveCard_Current);	
			//AMMO42_OUT_task();
			//AMMO17_OUT_task();
//////////////////////////////////////////////////////////////////////	
			test_rT[4]= GetSysTime_us ();
			test_rT[5] = (u32)(test_rT[4] - test_rT[3]) ;	//test_rT[5] rT��˼��RunTime ��1000HZʱ��Ƭ�������ִ��ʱ��
}

static void Loop_500Hz(void)	//2msִ��һ��   ��̬���ٶȻ���������
{	
			chassis_task();			
			referee_usart_task(); //����ϵͳ��ȡ
}

static void Loop_200Hz(void)	//5msִ��һ��  RPY���㡢��̬�ǶȻ�
{
//	/*��ȡ��̬�ǣ�ROLL PITCH YAW��*/
//	/*��̬�ǶȻ�����  ��̬����PID�⻷*/
		//UserTask();
//			Rescue_task();
//			AMMO_42mm_OUT_task();
//			AMMO_17mm_OUT_task();
}

static void Loop_100Hz(void)	//10msִ��һ�Σ�ִ��ʱ���ԼΪ20us  �߶��ٶȻ����߶Ȼ�
{
			test_rT[0]= GetSysTime_us ();

			test_rT[1]= GetSysTime_us ();
			test_rT[2] = (u32)(test_rT[1] - test_rT[0]) ;	//test_rT[2] rT��˼��RunTime ��100HZʱ��Ƭ�������ִ��ʱ��
				
}

static void Loop_50Hz(void)	//20msִ��һ��   λ���ٶȻ�
{	
}

static void Loop_20Hz(void)	//50msִ��һ��
{	
}

static void Loop_2Hz(void)	//500msִ��һ��
{
}

//ϵͳ�������ã�������ִͬ��Ƶ�ʵġ��̡߳�
static sched_task_t sched_tasks[] = 
{
	{Loop_1000Hz,1000,  0, 0},			//void(*task_func)(void);
	{Loop_500Hz , 500,  0, 0},			//uint16_t rate_hz;
	{Loop_200Hz , 200,  0, 0},			//uint16_t interval_ticks;
	{Loop_100Hz , 100,  0, 0},			//uint32_t last_run;
	{Loop_50Hz  ,  50,  0, 0},
	{Loop_20Hz  ,  20,  0, 0},
	{Loop_2Hz   ,   2,  0, 0},
};
//�������鳤�ȣ��ж��߳�����
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//��ʼ�������
	for(index=0;index < TASK_NUM;index++)
	{
		//����ÿ���������ʱ������
		sched_tasks[index].interval_ticks = TICK_PER_SECOND/sched_tasks[index].rate_hz; //  TICK_PER_SECOND = 1000
		//�������Ϊ1��Ҳ����1ms
		if(sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//��������ŵ�main������while(1)�У���ͣ�ж��Ƿ����߳�Ӧ��ִ��
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//ѭ���ж������̣߳��Ƿ�Ӧ��ִ��

	
	for(index=0;index < TASK_NUM;index++)
	{
		//��ȡϵͳ��ǰʱ�䣬��λMS
		uint32_t tnow = SysTick_GetTick();//��ȡ��systime_ms��ֵ����ֵ��Systick_Handler��ÿ1ms�ݼ�1
		//�����жϣ������ǰʱ���ȥ��һ��ִ�е�ʱ�䣬���ڵ��ڸ��̵߳�ִ�����ڣ���ִ���߳�
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			
			//�����̵߳�ִ��ʱ�䣬������һ���ж�
			sched_tasks[index].last_run = tnow;
			//ִ���̺߳�����ʹ�õ��Ǻ���ָ��
			sched_tasks[index].task_func();

		}	 
	}
	

}

