/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : usart_printf.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2020.11.18
  ����޸�   :
  ��������   : printf�ض���������ʾ����ʹ�ú�����
  �����б�   : 1) fputc()��printf�ض���			 ���ⲿֱ��ʹ��printf��
							 2)	Virtual_Oscilloscope()   	 ���ⲿ����:ʹ�ô���
							 3) Vcan_Sendware()			 			 ���ڲ����ã�Virtual_Oscilloscope()��
							 4) Usart_SendArray()				   ���ڲ����ã�Vcan_Sendware()��
							 5) Usart_SendByte()				   ���ڲ����ã�Usart_SendArray()��
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "usart_printf.h"

/* �ڲ��궨�� ----------------------------------------------------------------*/
#define US_huart  huart2   //printf������ʾ����ʹ�ô���

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

/* �ڲ�����ԭ������ ----------------------------------------------------------*/
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch );
void Usart_SendArray( UART_HandleTypeDef *huart, uint8_t *array, uint16_t num);
void Vcan_Sendware(uint8_t *wareaddr, uint32_t waresize);

/* �������岿�� --------------------------------------------------------------*/
//===================================================================================================================//
/*********************************************** �����ض���USART2��*************************************************/
//===================================================================================================================//
/**
  * @brief				�ض���c���printf����������2�ķ��ͺ���
  * @param[in]		
	* @param[out]		
  * @retval				
*/
int fputc(int ch, FILE* f)
{
		HAL_UART_Transmit(&US_huart, (uint8_t *)&ch, 1, 10); 
    return ch;
}

//===================================================================================================================//
/********************************************** ����ʾ������USART2�� *************************************************/
//===================================================================================================================//

/**
  * @brief				����ʾ�������κ���
  * @param[in]		date1-date8�������ӡ������
	* @param[out]		
  * @retval				none
*/
void Virtual_Oscilloscope(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8)
{
	float angle[8];	
	
	angle[0] = data1;
	angle[1] = data2;
	angle[2] = data3;
	angle[3] = data4;	
	angle[4] = data5;
	angle[5] = data6;
	angle[6] = data7;
	angle[7] = data8;
	Vcan_Sendware((uint8_t *)angle,sizeof(angle));
}

/**
  * @brief				����ʾ�������ͺ���
	* @param[in]		wareaddr��ָ���������ָ��
	* @param[out]		waresize���������ݳ���
  * @retval				none
*/
void Vcan_Sendware(uint8_t *wareaddr, uint32_t waresize)
{
		#define CMD_WARE     0x03
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����

    Usart_SendArray(&US_huart, cmdf, sizeof(cmdf));    //�ȷ���ǰ����
    Usart_SendArray(&US_huart, wareaddr, waresize);    //��������
    Usart_SendArray(&US_huart, cmdr, sizeof(cmdr));    //���ͺ�����
}

/******** ����һ���ַ� *********/
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch )
{
  HAL_UART_Transmit(huart, (uint8_t *)&ch, 1, 10); 
}
	
/******** ����8λ������ *********/
void Usart_SendArray( UART_HandleTypeDef *huart, uint8_t *array, uint16_t num)
{
	uint8_t i;
	for(i=0;i<num;i++)
	{
		Usart_SendByte(huart,array[i]);
	}		
}

