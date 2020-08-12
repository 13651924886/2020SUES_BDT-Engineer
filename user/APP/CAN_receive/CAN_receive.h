/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
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

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN1
#define XYZ_CAN CAN1

/* CAN send and receive ID */
typedef enum
{

//can1
		CAN_CHASSIS_ALL_ID = 0x200,	//CAN���ͱ��ı�ʶ��
		CAN_GIMBAL_ALL_ID = 0x1FF,	//CAN���ͱ��ı�ʶ��

    CAN_3508_M1_ID = 0x201,			//CAN�������ı�ʶ��
    CAN_3508_M2_ID = 0x202,			//CAN�������ı�ʶ��
    CAN_3508_M3_ID = 0x203,			//CAN�������ı�ʶ��
    CAN_3508_M4_ID = 0x204,			//CAN�������ı�ʶ��

    CAN_Y_MOTION_MOTOR_ID = 0x205,		//CAN�������ı�ʶ��
    CAN_Z_MOTION_MOTOR1_ID = 0x206,		//CAN�������ı�ʶ��
		CAN_Z_MOTION_MOTOR2_ID = 0x207,
	
//can2	
    CAN_GRIPPER_1_MOTOR_ID = 0x201,//CAN�������ı�ʶ��
		CAN_GRIPPER_2_MOTOR_ID = 0x202,//CAN�������ı�ʶ��
		CAN_GIMBAL_YAW_MOTOR_ID = 0x203,
		CAN_GIMBAL_PITCH_MOTOR_ID = 0x204,
		CAN_17AMMO_HATCH_MOTOR_ID = 0x205,
		CAN_42AMMO_HATCH_MOTOR_ID = 0x206,

} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t ecd;								//ת�ӻ�е�Ƕ�  0-8191��Ӧ0-360��
    int16_t speed_rpm;					//ת��ת�� ��λ��RPM
    int16_t given_current;			//���Ƶ��� -16384-0-16384��Ӧ-20-0-20A
    uint8_t temperate;					//����¶� ��λ����
    int16_t last_ecd;						//�ϴ�ת�ӻ�е�Ƕ�
} motor_measure_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//���͵��̵����������
extern void CAN1_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void CAN1_CMD_XYZ(int16_t Y_MOTION_motor, int16_t Z_MOTION_MOTOR1, int16_t Z_MOTION_MOTOR2, int16_t GPRIPPER_MOTOR2);

//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Y_MOTION_MOTOR_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Z_MOTION_MOTOR1_Measure_Point(void);
extern const motor_measure_t *get_Z_MOTION_MOTOR2_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_GRIPPER_1_MOTOR_Measure_Point(void);
extern const motor_measure_t *get_GRIPPER_2_MOTOR_Measure_Point(void);	
extern const motor_measure_t *get_YAW_MOTOR_Measure_Point(void);
extern const motor_measure_t *get_PITCH_MOTOR_Measure_Point(void);
extern const motor_measure_t *get_Ammo17_Hatch_MOTOR_Measure_Point(void);
extern const motor_measure_t *get_Ammo42_Hatch_MOTOR_Measure_Point(void);



#endif
