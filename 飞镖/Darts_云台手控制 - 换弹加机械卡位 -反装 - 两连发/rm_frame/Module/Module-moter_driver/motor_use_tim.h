#ifndef  __Motor_USE_TIM_H
#define  __Motor_USE_TIM_H
/* ����ͷ�ļ� ---------------------------------------------------------------*/
#include "myinclude.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨�� -------------------------------------------------*/
#define HIGH_SPEED 200
#define LOW_SPEED  100

/* ��ģ�����ⲿ�ṩ�Ľṹ��/ö�ٶ��� ----------------------------------------*/

/* ��ģ�����ⲿ�ṩ�ı������� -----------------------------------------------*/

/* ��ģ�����ⲿ�ṩ���Զ����������ͱ������� ---------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ---------------------------------------*/
void Shoot_Firction_Motor_PWM_Init(void);
void Shoot_Firction_Motor(uint32_t wheelone,uint32_t wheeltwo);
void Shoot_Firction_Motor_Stop(void);
void Shoot_Firction_Motor_Restart(void);


#endif
