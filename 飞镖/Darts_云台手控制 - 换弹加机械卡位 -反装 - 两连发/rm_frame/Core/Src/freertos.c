/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "myinclude.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId RemoteDataTaskHandle;
osThreadId VisionDataTaskHandle;
osThreadId RefereeDataTaskHandle;
osThreadId TriggerDriveTaskHandle;
osThreadId EnergyStorageTaskHandle;
osThreadId GimbalControlTaskHandle;
osThreadId ChassisControlTaskHandle;	
osThreadId OffLineCheckTaskHandle;
osThreadId PrintfTaskHandle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Remote_Data_Task(void const * argument);
void Vision_Data_Task(void const * argument);
void Referee_Data_Task(void const * argument);
void Energy_Storage_Task(void const * argument);
void Trigger_Drive_Task(void const * argument);
void Gimbal_Control_Task(void const * argument);
void Chassis_Control_Task(void const * argument);
void OffLine_Check_Task(void const *argument);
void Printf_Task(void const *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	/*********���ݽ����봦��*********/
	osThreadDef(RemoteDataTask, Remote_Data_Task, osPriorityHigh, 0, 256);          					 //ң��������
	RemoteDataTaskHandle = osThreadCreate(osThread(RemoteDataTask), NULL);
	
	osThreadDef(RefereeDataTask, Referee_Data_Task, osPriorityHigh, 0, 128);									 //����ϵͳ����
	RefereeDataTaskHandle = osThreadCreate(osThread(RefereeDataTask), NULL);
	
//	osThreadDef(VisionDataTask, Vision_Data_Task, osPriorityHigh, 0, 128);			 							 //�Ӿ�����
//	VisionDataTaskHandle = osThreadCreate(osThread(VisionDataTask), NULL);
	
	/**********�������**************/
	osThreadDef(EnergyStorageTask, Energy_Storage_Task, osPriorityAboveNormal, 0, 128);  	 	 //��������
	EnergyStorageTaskHandle = osThreadCreate(osThread(EnergyStorageTask),  NULL);
	
	osThreadDef(TriggerDriveTask, Trigger_Drive_Task, osPriorityAboveNormal, 0, 256);   		 	 //���̿�������
	TriggerDriveTaskHandle = osThreadCreate(osThread(TriggerDriveTask),  NULL);
	
	/**********���̿���**************/
//	osThreadDef(ChassisContrlTask, Chassis_Control_Task, osPriorityAboveNormal, 0, 256); 			 //���̿�������
//	ChassisControlTaskHandle = osThreadCreate(osThread(ChassisContrlTask), NULL); 
	
	/**********��̨����**************/
	osThreadDef(GimbalControlTask, Gimbal_Control_Task, osPriorityAboveNormal, 0, 256);  	     //��̨��������
	GimbalControlTaskHandle = osThreadCreate(osThread(GimbalControlTask), NULL);
	
	/**********���߼��**************/
	osThreadDef(OffLineCheckTask, OffLine_Check_Task, osPriorityNormal, 0, 128);							 //���߼������
	OffLineCheckTaskHandle = osThreadCreate(osThread(OffLineCheckTask), NULL);

	/**********��ӡ����**************/
	osThreadDef(PrintfTask, Printf_Task, osPriorityNormal, 0, 128);							 //���߼������
	PrintfTaskHandle = osThreadCreate(osThread(PrintfTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();	
  /* Infinite loop */
  for(;;)
  {
		GREEN_TOG();		
    osDelayUntil(&xLastWakeTime,1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
