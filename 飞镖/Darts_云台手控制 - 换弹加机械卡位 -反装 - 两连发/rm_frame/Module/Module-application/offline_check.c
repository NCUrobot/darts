/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : offline_check.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2020.12.13
  ����޸�   :
  ��������   : ���߼������TIM��ʼ�������߼����غ���
  �����б�   : 1) Off_Line_Detect_Init()						���ⲿ���ã�bsp.c��
							 2) Refresh_SysTime()									���ڲ����ã�HAL_TIM_PeriodElapsedCallback��
							 3) HAL_TIM_PeriodElapsedCallback()		��HAL�⺯����TIM�жϻص�������
							 //ʹ��CubeMX�޸Ĺ��̲����ɳ���ʱ��Ҫ��main�еĸú���ע��
							 4) Refresh_Device_OffLine_Time()			���ⲿ���ã�ʹ�ô���
							 5) Refresh_Task_OffLine_Time()				���ⲿ���ã�ʹ�ô���
							 6) Device_OffLine_Check()						���ⲿ���ã�offline_check_task.c��
							 7) Task_OffLine_Check()							���ⲿ���ã�offline_check_task.c��
	
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "offline_check.h"
/* �ڲ��궨�� ----------------------------------------------------------------*/
#define OL_htim    htim3//���߼���ʱ���ö�ʱ��

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/
Off_Line_TypeDef  off_line;

/* �ڲ����� ------------------------------------------------------------------*/
float g_Time_Device_OffLine[DeviceTotal_TOE] = {0};//�������һ��ͨ��ʱ������
float g_Time_Task_OffLine[TaskTotal_TOE] = {0};//�������һ��ͨ��ʱ������

/* �ڲ�����ԭ������ ----------------------------------------------------------*/
inline float Get_System_Timer(void);
void Refresh_SysTime(void);

/* �������岿�� --------------------------------------------------------------*/
/**
  * @brief				���߼���ʼ��
  * @param[out]		
  * @param[in]		
  * @retval				
*/
uint8_t Off_Line_Detect_Init(void)
{
	int state;
	off_line.enable = 0;
	off_line.state = 0;
	off_line.task = 0;
	off_line.time = 0;
	off_line.htim = &OL_htim;//��ʱ�����趨 ÿ 10us ��һ����  ����ֵΪ 100-1 (1ms)  ���� Timer3 ��Ƶ168M Ԥ��Ƶ (840-1) ����ֵ (100-1)
	state=HAL_TIM_Base_Start_IT(off_line.htim);//����ʱ�������
	
  return state;
}

/**
  * @brief				����ϵͳʱ�� ms(�ж�ˢ���е���)
  * @param[out]		
  * @param[in]		
  * @retval				
*/
void Refresh_SysTime(void)
{
		off_line.time += 1;
}
/**
  * @brief				TIM�жϻص���������Ҫ��main.c��cubemx���ɵ�TIM�жϻص�����ע�ͣ�
  * @param[out]		
  * @param[in]		
  * @retval				
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) 
	{
    HAL_IncTick();
  }

	else if(htim == (&OL_htim))
	{
		Refresh_SysTime();
	}	
}
/**
  * @brief				�������ǰ����ʱ��
  * @param[out]		
  * @param[in]		
  * @retval				
*/
float Get_System_Timer(void)
{
	return off_line.htim->Instance->CNT / 100.0 + off_line.time;   //�����㣬�õ�������ʱ�䵥λΪms
}

/**
  * @brief				ˢ������ͨ��ʱ������
  * @param[out]		
  * @param[in]		
  * @retval				
*/
void Refresh_Device_OffLine_Time(DeviceX_DEF DevX_TOE)
{	
	g_Time_Device_OffLine[DevX_TOE] = Get_System_Timer();	
}

/**
  * @brief				ˢ������ʱ������
  * @param[out]		
  * @param[in]		
  * @retval				
*/
void Refresh_Task_OffLine_Time(TaskX_DEF TaskX_TOE)
{	
	g_Time_Task_OffLine[TaskX_TOE] = Get_System_Timer();	
}

/**
  * @brief				������߼����
  * @param[out]		
  * @param[in]		
  * @retval				
*/
void Device_OffLine_Check(void)
{
	uint8_t num = 0;//��ʱ�����ۼ���
	float time = Get_System_Timer();//��ǰϵͳʱ��

	for(num = 0; num < DeviceTotal_TOE; num++)
	{
		if(time-g_Time_Device_OffLine[num] > OFFLINE_TIME)
		{
			MyFlagSet(off_line.device_offline_flag,(num));//���ö��߱�־
		}
		else
		{
			MyFlagClear(off_line.device_offline_flag,(num));//������߱�־
		}
	}
}

/**
  * @brief				������߼��
  * @param[out]		
  * @param[in]		
  * @retval				
*/
void Task_OffLine_Check(void)
{
	short num = 0;//��ʱ�����ۼ���
	float time = Get_System_Timer();//��ǰϵͳʱ��

	for(num = 0;num < TaskTotal_TOE;num++)
	{
		if(time-g_Time_Task_OffLine[num] > OFFLINE_TIME)
		{
			MyFlagSet(off_line.task_offline_flag,(num));//���ö��߱�־
		}
		else
		{
			MyFlagClear(off_line.task_offline_flag,(num));//������߱�־
		}
	}
}
