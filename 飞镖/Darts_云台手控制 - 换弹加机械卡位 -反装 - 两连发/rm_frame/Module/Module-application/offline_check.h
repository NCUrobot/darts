#ifndef __OFFLINE_CHECK_H
#define __OFFLINE_CHECK_H

/* ����ͷ�ļ� ---------------------------------------------------------------*/
#include "myinclude.h"
/* ��ģ�����ⲿ�ṩ�ĺ궨�� -------------------------------------------------*/
#define OFFLINE_TIME 100 //���߼��ʱ��

#define MyFlagSet(x,y) x=x|(0x00000001<<y) 			//���ñ�־λ  y�ڼ�λ
#define MyFlagClear(x,y) x=x&~(0x00000001<<y)		//�����־λ  y�ڼ�λ
#define MyFlagGet(x,y) (x&(0x00000001<<y))
/* ��ģ�����ⲿ�ṩ�Ľṹ��/ö�ٶ��� ----------------------------------------*/
typedef enum
{
		YawGimbalMotor_TOE,				//Yaw�������ݽ���
		PitchGimbalMotor_TOE,			//Pitch�������ݽ���
  	TriggerMotor_TOE,					//���̵�����ݽ���
		ChassisMotor_TOE,					//���̵�����ݽ���
		RemoteControl_TOE,				//ң�������ݽ���
		Referee_TOE,							//����ϵͳ���ݽ���
		Vision_TOE,								//�Ӿ����ݽ���
  
		DeviceTotal_TOE	
}DeviceX_DEF;

typedef enum
{
	RemoteDataTask_TOE = 0,	
	VisionDataTask_TOE,
	RefereeDataTask_TOE,
	FrictionDriveTask_TOE,
	TriggerDriveTask_TOE,
	GimbalContrlTask_TOE,
	ChassisControlTask_TOE,
	OutLineCheckTask_TOE,
	
	TaskTotal_TOE	
}TaskX_DEF;

typedef struct
{
	uint8_t mode;											//����ģʽ
	uint8_t enable;										//ʹ��
	uint8_t state;										//״̬
	uint8_t task;											//����
	uint32_t time;										//System run time mm
	TIM_HandleTypeDef *htim;					//ʱ����������
	uint8_t device_offline_flag;			//������߱�־
	uint8_t task_offline_flag;				//������߱�־
	
}Off_Line_TypeDef;
/* ��ģ�����ⲿ�ṩ�ı������� -----------------------------------------------*/
extern Off_Line_TypeDef  off_line;

/* ��ģ�����ⲿ�ṩ���Զ����������ͱ������� ---------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ---------------------------------------*/
uint8_t Off_Line_Detect_Init(void);
void Refresh_Device_OffLine_Time(DeviceX_DEF DevX_TOE);
void Refresh_Task_OffLine_Time(TaskX_DEF Task_TOE);
void Device_OffLine_Check(void);
void Task_OffLine_Check(void);



#endif



