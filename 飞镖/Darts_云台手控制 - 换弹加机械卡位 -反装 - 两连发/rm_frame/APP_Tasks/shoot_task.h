#ifndef  __SHOOT_FRICTION_TASK_H
#define  __SHOOT_FRICTION_TASK_H
/* ����ͷ�ļ� ---------------------------------------------------------------*/
#include "myinclude.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨�� -------------------------------------------------*/
#define REMOTE_CONTROL              1
#define ROBOT_INTERACTIVE_CONTROL   2

/* ��ģ�����ⲿ�ṩ�Ľṹ��/ö�ٶ��� ----------------------------------------*/
typedef struct
{
  int8_t  dart_control_mode;
  int8_t  storage_init_flag;
	float   trigger_current_value;
	int32_t trigger_get_pos;
	int32_t trigger_set_pos;
	int16_t trigger_get_spd;
	int16_t trigger_set_spd;
  int8_t  last_s1_state;
  int8_t  rotate_times;//�ڼ��η���
} shoot_control_t;	
/* ��ģ�����ⲿ�ṩ�ı������� -----------------------------------------------*/

/* ��ģ�����ⲿ�ṩ���Զ����������ͱ������� ---------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ---------------------------------------*/
extern shoot_control_t shoot_control;          //������ݽṹ��




#endif



