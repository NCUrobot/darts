/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       crc8_crc16.c/h
  * @brief      crc8 and crc16 calculate function, verify function, append function.
  *             crc8��crc16���㺯��,У�麯��,��Ӻ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef __CRC_H
#define __CRC_H

#include "main.h"

/**
  * @brief          calculate the crc8  
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      ucCRC8: init CRC8
  * @retval         calculated crc8
  */
/**
  * @brief          ����CRC8
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @param[in]      ucCRC8:��ʼCRC8
  * @retval         �������CRC8
  */
uint8_t Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);

/**
  * @brief          CRC8 verify function  
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         true of false
  */
/**
  * @brief          CRC8У�麯��
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         ����߼�
  */
uint32_t Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

/**
  * @brief          append CRC8 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
/**
  * @brief          ���CRC8�����ݵĽ�β
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         none
  */
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

/**
  * @brief          calculate the crc16  
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      wCRC: init CRC16
  * @retval         calculated crc16
  */
/**
  * @brief          ����CRC16
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @param[in]      wCRC:��ʼCRC16
  * @retval         �������CRC16
  */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

/**
  * @brief          CRC16 verify function  
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         true of false
  */
/**
  * @brief          CRC16У�麯��
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         ����߼�
  */
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/**
  * @brief          append CRC16 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
/**
  * @brief          ���CRC16�����ݵĽ�β
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         none
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif

