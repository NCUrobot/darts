/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : remote_control_task.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2020.12.1
  ����޸�   : 
  ��������   : ң�������ݽ�һ���������񣬰����ж��Ǽ����������ң���������ȡ�
							 ����USART1������һ������ʱ�ڿ����жϽ��н��룬����������֪ͨ,Ȼ��
							 �����񷽿����У����������������״̬������ʱ��Ϊ4294967295ms��
							 Լ����47��17Сʱ����
  �����б�   : 1) Remote_Data_Task()      ��FreeRTOS����������ϵͳ����������С�  
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "remote_control_task.h"
#include "offline_check.h"
#include "remote_control.h"
/* �ڲ��궨�� ----------------------------------------------------------------*/

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/
remote_control_t  remote_control;
/* �ڲ����� ------------------------------------------------------------------*/

/* �ڲ�����ԭ������ ----------------------------------------------------------*/

/* �������岿�� --------------------------------------------------------------*/
/**
  * @brief				ң��������
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
		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //δ������֪ͨ��������״̬ȥ�ȴ�����֪ͨ
		
    if(NotifyValue == 1)
		{
			Refresh_Task_OffLine_Time(RemoteDataTask_TOE);
      
      //s1���ϲ�Ȼ���˻��м�times_value��һ�����²��˻��м�times_value��һ
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
        //�޷�
        if(remote_control.s1_times_value <= 0) 
            remote_control.s1_times_value = 0;
        else if(remote_control.s1_times_value >= 4)
            remote_control.s1_times_value = 4;

      
      //s2���ϲ�Ȼ���˻��м�times_value��һ�����²��˻��м�times_value��һ      
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
