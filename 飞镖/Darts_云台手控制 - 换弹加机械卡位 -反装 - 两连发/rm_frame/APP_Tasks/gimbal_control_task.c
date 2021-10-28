/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : gimbal_control_task.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2021.7.1
  ����޸�   : 
  ��������   : ������̨�����������Ƹ��콵���������Ʒ��ڸ�����
  �����б�   : 1) Gimbal_Control_Task()			��FreeRTOS����������ϵͳ����������С�
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "gimbal_control_task.h"
#include "offline_check.h"
#include "pid.h"
#include "motor_use_can.h"
#include "usart_printf.h"
#include "remote_control.h"

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

/* �ڲ�����ԭ������ ----------------------------------------------------------*/
void Gimbal_Motor_PID_Init(void);   

/* �������岿�� --------------------------------------------------------------*/
/**
  * @brief				��̨����(���Ƶ��Ƹ�������ʵ�ֵ��ڸ���)
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
  * @brief				PID��ʼ��������6020��
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Gimbal_Motor_PID_Init(void)   
{
	PID_Param_Init(&motor_pid[PID_PITCH_MOTOR_POS], POSITION_PID, 30000, 30000,
									20.0f, 0.001f, 0.25f);
	
}

