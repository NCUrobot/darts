#ifndef  __REMOTE_CONTORL_TASK_H
#define	 __REMOTE_CONTORL_TASK_H
/* 包含头文件 ---------------------------------------------------------------*/
#include "myinclude.h"

/* 本模块向外部提供的宏定义 -------------------------------------------------*/

/* 本模块向外部提供的结构体/枚举定义 ----------------------------------------*/
typedef struct
{
  int8_t  last_s1_state;
  int8_t  last_s2_state;
  int8_t  s1_times_value;
  int8_t  s2_times_value;
} remote_control_t;
/* 本模块向外部提供的变量声明 -----------------------------------------------*/

/* 本模块向外部提供的自定义数据类型变量声明 ---------------------------------*/

/* 本模块向外部提供的接口函数原型声明 ---------------------------------------*/
extern remote_control_t  remote_control;







#endif
