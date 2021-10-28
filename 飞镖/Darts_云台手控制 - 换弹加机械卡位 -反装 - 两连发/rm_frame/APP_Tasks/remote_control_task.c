/*******************************************************************************
                      版权所有 (C), 2020-,NCUROBOT
 *******************************************************************************
  文 件 名   : remote_control_task.c
  版 本 号   : V1.0
  作    者   : 高云海
  生成日期   : 2020.12.1
  最近修改   : 
  功能描述   : 遥控器数据进一步处理任务，包括判断是键鼠操作还是遥控器操作等。
							 （当USART1接收完一组数据时在空闲中断进行解码，并发送任务通知,然后
							 该任务方可运行，否则该任务处于阻塞状态，阻塞时间为4294967295ms即
							 约等于47天17小时。）
  函数列表   : 1) Remote_Data_Task()      【FreeRTOS函数：操作系统任务调度运行】  
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "remote_control_task.h"
#include "offline_check.h"
#include "remote_control.h"
/* 内部宏定义 ----------------------------------------------------------------*/

/* 内部自定义数据类型的变量 --------------------------------------------------*/
remote_control_t  remote_control;
/* 内部变量 ------------------------------------------------------------------*/

/* 内部函数原型声明 ----------------------------------------------------------*/

/* 函数主体部分 --------------------------------------------------------------*/
/**
  * @brief				遥控器任务
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Remote_Data_Task(void const * argument)
{
	  uint32_t NotifyValue;

//		portTickType xLastWakeTime;
//		xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //未有任务通知则进入堵塞状态去等待任务通知
		
    if(NotifyValue == 1)
		{
			Refresh_Task_OffLine_Time(RemoteDataTask_TOE);
      
      //s1向上拨然后退回中间times_value加一，向下拨退回中间times_value减一
      if(rc_ctrl.rc.s1 == LEFT_S1_UP || rc_ctrl.rc.s1 == LEFT_S1_DOWN)
      { 
          remote_control.last_s1_state = rc_ctrl.rc.s1;
      }
      if(remote_control.last_s1_state == LEFT_S1_UP && rc_ctrl.rc.s1 == LEFT_S1_MID)
      {        
          remote_control.s1_times_value++;
          remote_control.last_s1_state = rc_ctrl.rc.s1;        
      }
      else if(remote_control.last_s1_state == LEFT_S1_DOWN && rc_ctrl.rc.s1 == LEFT_S1_MID)
      {        
          remote_control.s1_times_value--;
          remote_control.last_s1_state = rc_ctrl.rc.s1;        
      }
        //限幅
        if(remote_control.s1_times_value <= 0) 
            remote_control.s1_times_value = 0;
        else if(remote_control.s1_times_value >= 4)
            remote_control.s1_times_value = 4;

      
      //s2向上拨然后退回中间times_value加一，向下拨退回中间times_value减一      
      if(rc_ctrl.rc.s2 == RIGHT_S2_UP || rc_ctrl.rc.s2 == RIGHT_S2_DOWN)
      { 
          remote_control.last_s2_state = rc_ctrl.rc.s2;
      }
      if(remote_control.last_s2_state == RIGHT_S2_UP && rc_ctrl.rc.s2 == RIGHT_S2_MID)
      {        
          remote_control.s2_times_value++;
          remote_control.last_s2_state = rc_ctrl.rc.s2;        
      }
      else if(remote_control.last_s2_state == RIGHT_S2_DOWN && rc_ctrl.rc.s2 == RIGHT_S2_MID)
      {        
          remote_control.s2_times_value--;
          remote_control.last_s2_state = rc_ctrl.rc.s2;        
      }
			
		}
		
//		osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}
