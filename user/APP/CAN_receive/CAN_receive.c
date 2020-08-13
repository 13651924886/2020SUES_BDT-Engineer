/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2019-11-16     	Stone              1. ���
  *
  @verbatim
  ==============================================================================
	�������̽��ͣ�
	
		�����CAN1���߷��ͷ������� ---->CAN1_RX0_IRQHandler() ----> CAN1_hook() ----> get_motor_measure(ptr, rx_message)
																								 �� get_gimbal_motor_measuer(ptr, rx_message)
																								 
		�����CAN2���߷��ͷ������� ---->CAN2_RX0_IRQHandler() ----> CAN1_hook() ----> get_motor_measure(ptr, rx_message)
																								 �� get_gimbal_motor_measuer(ptr, rx_message)
						������Ϣ�ֱ�����motor_yaw, motor_pit, motor_trigger, motor_chassis[4]�ṹ�������
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

/************************* ͷ�ļ� ******************************/

#include "CAN_Receive.h"

#include "stm32f4xx.h"
#include "rng.h"

#include "Detect_Task.h"


/*********************** �궨�庯�� ****************************/

//���̵�����ݶ�ȡ
//��CAN1_hook()��ʹ��
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//��̨������ݶ�ȡ
//CAN_hook()��ʹ��
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
		
		
/************************ �������� ******************************/
		
//���Ľ�������
//CAN1_RX0_IRQHandler()��ʹ��
//CAN2_RX0_IRQHandler()��ʹ��
static void CAN1_hook(CanRxMsg *rx_message);
static void CAN2_hook(CanRxMsg *rx_message);

		
		
		
/*********************** ȫ�ֱ������� ***************************/
//�����������
//static motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_chassis[4];
motor_measure_t Y_motion_motor,Z_motion_motor1, Z_motion_motor2, gripper_1_motor, 
		gripper_2_motor,gimbal_yaw_motor,gimbal_pitch_motor,motor_chassis[4],
		ammo17_hatch_motor,ammo42_hatch_motor; //����Debug�鿴���� �Ѿ�̬ȥ��

static CanTxMsg GIMBAL_TxMessage;
		
		
		
		
/************************ �������� ******************************/
//can1�����жϷ�����
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN1_hook(&rx1_message);
    }
}

//can2�����жϷ�����
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN2_hook(&rx2_message);
    }
}


void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}
//CAN ���� 0x700��ID�����ݣ�������M3508�����������IDģʽ
//��������û��ʹ��
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN2, &TxMessage);
}

//���͵��̵����������
//��chassis_task��ʹ��
void CAN1_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}


void CAN1_CMD_XYZ(int16_t Y_MOTION_motor, int16_t Z_MOTION_MOTOR1, int16_t Z_MOTION_MOTOR2, int16_t GPRIPPER_MOTOR2)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (Y_MOTION_motor >> 8);
    GIMBAL_TxMessage.Data[1] = Y_MOTION_motor;
    GIMBAL_TxMessage.Data[2] = (Z_MOTION_MOTOR1 >> 8);
    GIMBAL_TxMessage.Data[3] = Z_MOTION_MOTOR1;
    GIMBAL_TxMessage.Data[4] = (Z_MOTION_MOTOR2 >> 8);
    GIMBAL_TxMessage.Data[5] = Z_MOTION_MOTOR2;
    GIMBAL_TxMessage.Data[6] = (GPRIPPER_MOTOR2 >> 8);
    GIMBAL_TxMessage.Data[7] = GPRIPPER_MOTOR2;
    CAN_Transmit( XYZ_CAN,  &GIMBAL_TxMessage );
}

void CAN2_CMD_GRIPPER(int16_t GRIPPER_MOTOR1, int16_t GRIPPER_MOTOR2, int16_t NONE1, int16_t NONE2)
{
    GIMBAL_TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (GRIPPER_MOTOR1 >> 8);
    GIMBAL_TxMessage.Data[1] = GRIPPER_MOTOR1;
    GIMBAL_TxMessage.Data[2] = (GRIPPER_MOTOR2 >> 8);
    GIMBAL_TxMessage.Data[3] = GRIPPER_MOTOR2;
    GIMBAL_TxMessage.Data[4] = (NONE1 >> 8);
    GIMBAL_TxMessage.Data[5] = NONE1;
    GIMBAL_TxMessage.Data[6] = (NONE2 >> 8);
    GIMBAL_TxMessage.Data[7] = NONE2;
    CAN_Transmit( GRIPPER_CAN,  &GIMBAL_TxMessage );
}

//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
  const motor_measure_t *get_Y_MOTION_MOTOR_Measure_Point(void)
{
    return &Y_motion_motor;
}
//����XYZ��̨��Z����1������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Z_MOTION_MOTOR1_Measure_Point(void)
{
    return &Z_motion_motor1;
}
//����XYZ��̨��Z����2������ַ
const motor_measure_t *get_Z_MOTION_MOTOR2_Measure_Point(void)
{
    return &Z_motion_motor2;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];//i & 0000 0011
}

//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_GRIPPER_1_MOTOR_Measure_Point(void)
{
    return &gripper_1_motor;
}
const motor_measure_t *get_GRIPPER_2_MOTOR_Measure_Point(void)
{
		return &gripper_2_motor;
}
const motor_measure_t *get_YAW_MOTOR_Measure_Point(void)
{
		return &gimbal_yaw_motor;
}
const motor_measure_t *get_PITCH_MOTOR_Measure_Point(void)
{
		return &gimbal_pitch_motor;
}
const motor_measure_t *get_Ammo17_Hatch_MOTOR_Measure_Point(void)
{
		return &ammo17_hatch_motor;
}
const motor_measure_t *get_Ammo42_Hatch_MOTOR_Measure_Point(void)
{
		return &ammo42_hatch_motor;
}


//CAN���ߵ���������Ľ������������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
//��ʱȥ��DetectTask����
static void CAN1_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_Y_MOTION_MOTOR_ID:
    {
        //���������ݺ꺯��
        get_motor_measure(&Y_motion_motor, rx_message);
        //��¼ʱ��
        //DetectHook(YawGimbalMotorTOE);  //��ʱȥ��DetectTask����
        break;
    }
    case CAN_Z_MOTION_MOTOR1_ID:
    {
        //���������ݺ꺯��
       get_motor_measure(&Z_motion_motor1, rx_message);
       // DetectHook(PitchGimbalMotorTOE);
        break;
    }
    case CAN_Z_MOTION_MOTOR2_ID:
    {
        //���������ݺ꺯��
        get_motor_measure(&Z_motion_motor2, rx_message);
        //��¼ʱ��
        //DetectHook(TriggerMotorTOE);  //��ʱȥ��DetectTask����
        break;
    }
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //������ID��
        i = rx_message->StdId - CAN_3508_M1_ID;
        //���������ݺ꺯��
        get_motor_measure(&motor_chassis[i], rx_message);
        //��¼ʱ��
        //DetectHook(ChassisMotor1TOE + i); //��ʱȥ��DetectTask����
        break;
    }

    default:
    {
        break;
    }
    }
}

static void CAN2_hook(CanRxMsg *rx_message)
{
	   switch (rx_message->StdId)
    {
			case CAN_GRIPPER_1_MOTOR_ID:
			{
					get_motor_measure(&gripper_1_motor, rx_message);
					break;
			}
			case CAN_GRIPPER_2_MOTOR_ID:
			{
					get_motor_measure(&gripper_2_motor, rx_message);
					break;
			}
			case CAN_GIMBAL_YAW_MOTOR_ID:
			{
					get_motor_measure(&gimbal_yaw_motor, rx_message);
					break;
			}
			case CAN_GIMBAL_PITCH_MOTOR_ID:
			{
					get_motor_measure(&gimbal_pitch_motor, rx_message);
					break;
			}
			case CAN_17AMMO_HATCH_MOTOR_ID:
			{
					get_motor_measure(&ammo17_hatch_motor, rx_message);
					break;
			}
			case CAN_42AMMO_HATCH_MOTOR_ID:
			{
					get_motor_measure(&ammo42_hatch_motor, rx_message);
					break;
			}	
			default:
			{
					break;
			}			
		}
}

