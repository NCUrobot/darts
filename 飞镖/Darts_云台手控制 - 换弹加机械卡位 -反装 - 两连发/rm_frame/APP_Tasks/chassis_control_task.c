/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : chassis_control_task.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2020.12.10
  ����޸�   : 
  ��������   : �������񣬴�����
  �����б�   : 1) Chassis_Control_Task()		��FreeRTOS����������ϵͳ����������С�
							 2) Chassis_Motor_PID_Init()	���ڲ�������Chassis_Control_Task���á�
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "chassis_control_task.h"
#include "pid.h"
#include "encoder.h"
#include "offline_check.h"
#include "motor_use_can.h"
#include "remote_control.h"

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

/* �ڲ�����ԭ������ ----------------------------------------------------------*/

/* �������岿�� --------------------------------------------------------------*/
/**
  * @brief				��������
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




