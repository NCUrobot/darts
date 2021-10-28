/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : bsp_tim.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2020.12.8
  ����޸�   : 
			 								
	��������   : ���ñ�����TIM��ʼ����TIM��������������ⷽ������IO���ⲿ�жϻص�����
  �����б�   : 1) Encoder_Mini512_TIM_Init()			���ⲿ���ã�bsp.c��
							 2) Get_TIM_COUNTER()								���ⲿ���ã�chassis_control_task.c��
							 3) HAL_GPIO_EXTI_Callback()				��HAL�⺯�����ⲿ�жϻص�������				
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "encoder.h"
/* �ڲ��궨�� ----------------------------------------------------------------*/
#define EM_htim    htim2//�������������ö�ʱ��

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

/* �ڲ�����ԭ������ ----------------------------------------------------------*/

/* �������岿�� --------------------------------------------------------------*/
/**
  * @brief				mini512��������ʼ��
  * @param[out]		
  * @param[in]		
  * @retval				
*/
void Encoder_Mini512_TIM_Init(void)
{
	HAL_TIM_Base_Start(&EM_htim);				//������ʱ�������ⲿ�������
}

/**
  * @brief				��ȡtim�ļ���ֵ
  * @param[out]		
  * @param[in]		
  * @retval				
*/
int32_t Get_TIM_COUNTER(void)
{	
	return  __HAL_TIM_GET_COUNTER(&EM_htim);	
}

/**
  * @brief				�ⲿ�жϻص���������������ת������
  * @param[out]		
  * @param[in]		
  * @retval				
*/
void	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == Encoder_Direction_Pin)
	{
		if(HAL_GPIO_ReadPin(Encoder_Direction_GPIO_Port,Encoder_Direction_Pin)==1)
			EM_htim.Instance->CR1 &=~(1<<4);//��CR1�Ĵ�����DIRλ��0��������������������
			
		else if(HAL_GPIO_ReadPin(Encoder_Direction_GPIO_Port,Encoder_Direction_Pin)==0)
			EM_htim.Instance->CR1 |=(1<<4); //��CR1�Ĵ�����DIRλ��1�����������ݼ�������
	}
}

