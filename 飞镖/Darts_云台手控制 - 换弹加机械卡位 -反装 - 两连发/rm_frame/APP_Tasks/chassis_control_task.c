/*******************************************************************************
                      版权所有 (C), 2020-,NCUROBOT
 *******************************************************************************
  文 件 名   : chassis_control_task.c
  版 本 号   : V1.0
  作    者   : 高云海
  生成日期   : 2020.12.10
  最近修改   : 
  功能描述   : 底盘任务，待完善
  函数列表   : 1) Chassis_Control_Task()		【FreeRTOS函数：操作系统任务调度运行】
							 2) Chassis_Motor_PID_Init()	【内部函数：Chassis_Control_Task调用】
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_control_task.h"
#include "pid.h"
#include "encoder.h"
#include "offline_check.h"
#include "motor_use_can.h"
#include "remote_control.h"

/* 内部宏定义 ----------------------------------------------------------------*/

/* 内部自定义数据类型的变量 --------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

/* 内部函数原型声明 ----------------------------------------------------------*/

/* 函数主体部分 --------------------------------------------------------------*/
/**
  * @brief				底盘任务
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Chassis_Control_Task(void const * argument)
{

	
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		Refresh_Task_OffLine_Time(ChassisControlTask_TOE);
		
		osDelayUntil(&xLastWakeTime,CHASSIS_PERIOD);		

	}

}




