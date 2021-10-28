/*******************************************************************************
                      版权所有 (C), 2020-,NCUROBOT
 *******************************************************************************
  文 件 名   : gimbal_control_task.c
  版 本 号   : V1.0
  作    者   : 高云海
  生成日期   : 2021.7.1
  最近修改   : 
  功能描述   : 飞镖云台任务：驱动电推杆伸降，进而控制飞镖俯仰角
  函数列表   : 1) Gimbal_Control_Task()			【FreeRTOS函数：操作系统任务调度运行】
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_control_task.h"
#include "offline_check.h"
#include "pid.h"
#include "motor_use_can.h"
#include "usart_printf.h"
#include "remote_control.h"

/* 内部宏定义 ----------------------------------------------------------------*/

/* 内部自定义数据类型的变量 --------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

/* 内部函数原型声明 ----------------------------------------------------------*/
void Gimbal_Motor_PID_Init(void);   

/* 函数主体部分 --------------------------------------------------------------*/
/**
  * @brief				云台任务(控制点推杆伸缩，实现调节俯仰)
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Gimbal_Control_Task(void const * argument)
{

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		Refresh_Task_OffLine_Time(GimbalContrlTask_TOE);
		
    if(rc_ctrl.rc.ch1 > 500)
    {
       IN1_OFF();
       IN2_ON();
       IN3_OFF();
       IN4_ON();             
    }
    
    else if(rc_ctrl.rc.ch1 < -500)
    {
       IN1_ON();
       IN2_OFF();
       IN3_ON();
       IN4_OFF();         
    }
    
    else if(rc_ctrl.rc.ch1 <= 300 && rc_ctrl.rc.ch1 >= -300)
    {
       IN1_OFF();
       IN2_OFF();
       IN3_OFF();
       IN4_OFF();      
    }


		osDelayUntil(&xLastWakeTime,GIMBAL_PERIOD);		

	}

}

/**
  * @brief				PID初始化函数（6020）
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Gimbal_Motor_PID_Init(void)   
{
	PID_Param_Init(&motor_pid[PID_PITCH_MOTOR_POS], POSITION_PID, 30000, 30000,
									20.0f, 0.001f, 0.25f);
	
}

