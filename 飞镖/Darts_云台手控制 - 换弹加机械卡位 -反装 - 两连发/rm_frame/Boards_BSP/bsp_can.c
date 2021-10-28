/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : bsp_can.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2020.11.18
  ����޸�   :
  ��������   : CANͨ�ų�ʼ��������CAN������CAN�����жϡ�CANͨ���˲�����ʼ����
  �����б�   : 1) BSP_CAN_Init() 			���ⲿ���ã�bsp.c��   
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_can.h"

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

/* �ڲ�����ԭ������ ----------------------------------------------------------*/

/* �������岿�� --------------------------------------------------------------*/
/**
  * @brief				CAN��ʼ������
  * @param[out]		
  * @param[in]		
  * @retval				
*/
uint8_t BSP_CAN_Init(CAN_HandleTypeDef* hcan)    
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	/*can��������ʼ��*/	
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//����ģʽ
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//����
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32λģʽ
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//���ӵ�fifo0
	canFilter.SlaveStartFilterBank=14;												//can2ɸѡ����ʼ���		
	
	if(hcan == &hcan1)
  {
    canFilter.FilterBank = 0;    														//ɸѡ����0
  }
  if(hcan == &hcan2)
  {
    canFilter.FilterBank = 14;															//ɸѡ����14
  }
	
	status|=HAL_CAN_ConfigFilter(hcan,&canFilter);					//���ù�����
	
	/*����CAN��CAN�ж�*/
	HAL_CAN_Start(hcan);					
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 ����fifo 0��Ϊ���ж�	
	
	return status;
}

