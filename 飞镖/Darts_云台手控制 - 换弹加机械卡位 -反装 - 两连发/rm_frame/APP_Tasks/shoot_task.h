#ifndef  __SHOOT_FRICTION_TASK_H
#define  __SHOOT_FRICTION_TASK_H
/* 包含头文件 ---------------------------------------------------------------*/
#include "myinclude.h"

/* 本模块向外部提供的宏定义 -------------------------------------------------*/
#define REMOTE_CONTROL              1
#define ROBOT_INTERACTIVE_CONTROL   2

/* 本模块向外部提供的结构体/枚举定义 ----------------------------------------*/
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
  int8_t  rotate_times;//第几次发射
} shoot_control_t;	
/* 本模块向外部提供的变量声明 -----------------------------------------------*/

/* 本模块向外部提供的自定义数据类型变量声明 ---------------------------------*/

/* 本模块向外部提供的接口函数原型声明 ---------------------------------------*/
extern shoot_control_t shoot_control;          //射击数据结构体




#endif



