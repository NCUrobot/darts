/*******************************************************************************
                      ��Ȩ���� (C), 2020-,NCUROBOT
 *******************************************************************************
  �� �� ��   : shoot_task.c
  �� �� ��   : V1.0
  ��    ��   : ���ƺ�
  ��������   : 2021.7.1
  ����޸�   :
  ��������   : ���ڷ������񣬰������������벦�̣����ڻ���������
  �����б�   : 1) Energy_Storage_Task()		��FreeRTOS����������ϵͳ����������С�
							 2) Trigger_Drive_Task()		��FreeRTOS����������ϵͳ����������С�
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "shoot_task.h"
#include "motor_use_tim.h"
#include "remote_control.h"
#include "remote_control_task.h"
#include "motor_use_can.h"
#include "offline_check.h"
#include "referee.h"
#include "pid.h"

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* �ڲ��Զ����������͵ı��� --------------------------------------------------*/
shoot_control_t shoot_control;   //�������ݽṹ��

/* �ڲ����� ------------------------------------------------------------------*/
static  int8_t  storage_state = 0;//������ɱ�־λ��0δ��ɣ�1����ɡ�
static  int8_t  pid_calc_flag = 0;//���Ƿ����PID�����־λ��1Ϊ���㣻2Ϊ�����㡿
static  int8_t  storage_times = 0;//������ɴ�������ʼΪ0��
static  int8_t  run_state = 2;//������ת����1���ϣ�2���¡�
static  int16_t delay_state = 0;//��ʱ��־λ�����ڴ������ȫ���º������ٿ�ʼ���ϸ�λ��
static  int16_t release_delay_state = 0;//�ͷź���ʱ��־λ��������ȫ�ͷź��ٽ�����һ�����ܡ�
static  int16_t second_delay_time = 0;//�������������м��ӳ�ʱ��
static  int8_t  get_dart_cmd_flag1 = 1;//��ȡ����ϵͳ���ڵ�һ��ָ���־λ
static  int8_t  get_dart_cmd_flag2 = 1;//��ȡ����ϵͳ���ڵ�����ָ���־λ
static  float   current_set_value = 0;
static  int16_t get_speed;
static  int16_t set_speed;
static  int32_t get_pos;

/* �ڲ�����ԭ������ ----------------------------------------------------------*/
void Trigger_Motor_PID_Init(void);
void EnergyStorage_Motor_PID_Init(void);
void Energy_Storage_First(void);
void Energy_Storage(void); 
void Energy_Release(void);


/* �������岿�� --------------------------------------------------------------*/
//===================================================================================================================//
/******************************************************Ħ����*********************************************************/
//===================================================================================================================//
/**
  * @brief				��������
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Energy_Storage_Task(void const *argument)
{  
  EnergyStorage_Motor_PID_Init();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();         
	
	for(;;)  
	{
		Refresh_Task_OffLine_Time(FrictionDriveTask_TOE);//��¼�������е�ʱ���
    get_pos = motor_get[CHASSIS_MOTOR_RF].total_angle;    
		get_speed = motor_get[CHASSIS_MOTOR_RF].speed_rpm;
 
    
  if(shoot_control.storage_init_flag == 0)
  {  
    //ģʽѡ��
    if(rc_ctrl.rc.s2 == RIGHT_S2_UP)//ң�ؿ���ģʽ
    {
      shoot_control.dart_control_mode = REMOTE_CONTROL;
      Energy_Storage_First();
    }
    else if(rc_ctrl.rc.s2 == RIGHT_S2_DOWN)//�����˽�������
    {
      shoot_control.dart_control_mode = ROBOT_INTERACTIVE_CONTROL;
      Energy_Storage_First();
    }  
  }
  else if(shoot_control.storage_init_flag == 1)
  {
    //ң�ؿ���ģʽ
    if(shoot_control.dart_control_mode == REMOTE_CONTROL) 
    {
      shoot_control.rotate_times = remote_control.s1_times_value;
      if(shoot_control.rotate_times == storage_times)
      {
        Energy_Release();
        release_delay_state++;
        if(release_delay_state >= 300 && storage_times < 4)//�ȴ���ȫ�ͷŽ�����һ������
        {
          Energy_Storage();
          if(storage_state ==1)  release_delay_state = 0;
        }
      }
      if(storage_times == 2 && storage_state == 1 && remote_control.s1_times_value == 1)
      {        
        second_delay_time++;
        if(second_delay_time >= 200)
        {        
          remote_control.s1_times_value = 2;
          second_delay_time = 0;
        }
      }
      if(storage_times == 4 && storage_state == 1 && remote_control.s1_times_value == 3)
      {        
        second_delay_time++;
        if(second_delay_time >= 200)
        {        
          remote_control.s1_times_value = 4;
          second_delay_time = 0;
        }
      }    
    }
    //�����˽�������
    else if(shoot_control.dart_control_mode == ROBOT_INTERACTIVE_CONTROL)
    {
      
      if(referee_used.dart_launch_num == 1 && get_dart_cmd_flag1 == 1)   
      {        
        shoot_control.rotate_times = referee_used.dart_launch_num;
        get_dart_cmd_flag1 = 0;
      }
      if(referee_used.dart_launch_num == 3 && get_dart_cmd_flag2 == 1)   
      {        
        shoot_control.rotate_times = referee_used.dart_launch_num;
        get_dart_cmd_flag2 = 0;
      }      
      
      if(shoot_control.rotate_times == storage_times)
      {
        Energy_Release();
        release_delay_state++;        
        if(release_delay_state >= 300 && storage_times < 4)//�ȴ���ȫ�ͷŽ�����һ������
        {
          Energy_Storage();
          if(storage_state ==1)  release_delay_state = 0;
        }
      } 
      if(storage_times == 2 && storage_state == 1 && shoot_control.rotate_times == 1)
      {        
        second_delay_time++;
        if(second_delay_time >= 200)
        {        
          shoot_control.rotate_times = 2;
          second_delay_time = 0;
        }
      }
      if(storage_times == 4 && storage_state == 1 && shoot_control.rotate_times == 3)
      {        
        second_delay_time++;
        if(second_delay_time >= 200)
        {        
          shoot_control.rotate_times = 4;
          second_delay_time = 0;
        }
      }      
    } 
  }    
    
    //PID����
    if(pid_calc_flag == 1)   
    {  
      current_set_value = PID_Calc(&motor_pid[PID_CHASSIS_MOTOR_RF_SPD], get_speed, set_speed);
    }
    
		Chassis_Motor_Drive(&hcan1,current_set_value,0,0,0);
   
		osDelayUntil(&xLastWakeTime,Energy_PERIOD);		
	}
}

/**
  * @brief				��ʼ���ܣ���һ�����ڵ�����
  * @param[in]		
	* @param[out]		
  * @retval				
*/
void Energy_Storage_First(void) 
{
  if(storage_state == 0)//����δ���
  {    
    if(run_state == 2)//������������
    {
      if(get_pos >  -1110000)
      {
          Shoot_Firction_Motor(150,150);//�����
          if(get_pos < -1000000)
          {        
            set_speed = -1000;
            pid_calc_flag = 1;        
          }
          else
          {        
            current_set_value = -5000;
            pid_calc_flag = 2;    
          }
      }
      else if(get_pos <= -1110000)
      {
          Shoot_Firction_Motor(120,120);//����ر�
          pid_calc_flag = 1;
          set_speed = 0;//��ס����
          delay_state ++;
          if(delay_state > 100)
          {
              run_state = 1;//�������ϸ�λ
              delay_state = 0;
          }                  
      }              
    }
    else if(run_state == 1)//�������ϸ�λ
    {
      if(get_pos < 0)
      {
          if(get_pos > -150000)
          {
              set_speed = 800;
              pid_calc_flag = 1;
          }
          else
          {
              current_set_value = 2500;
              pid_calc_flag = 2;
          }
      }
      else if(get_pos >= 0)
      {
          pid_calc_flag = 1;
          set_speed = 0;
          run_state = 2;//������ת�����־λ��λ
          shoot_control.storage_init_flag = 1;
          storage_times++;
          storage_state = 1;//�������            
      }
    }        
  }
}
/**
  * @brief				�������ܣ��������ڵ�����
  * @param[in]		
	* @param[out]		
  * @retval				
*/
void Energy_Storage(void) 
{    
  if(storage_state == 0)//����δ���
  {
    if(run_state == 2)//������������
    {
        if(get_pos >  -1110000)
        {
            Shoot_Firction_Motor(150,150);//�����
            if(get_pos < -1000000)
            {        
              set_speed = -1000;
              pid_calc_flag = 1;        
            }
            else
            {        
              current_set_value = -5000;
              pid_calc_flag = 2;    
            }
        }
        else if(get_pos <= -1110000)
        {
            Shoot_Firction_Motor(120,120);//����ر�
            pid_calc_flag = 1;
            set_speed = 0;//��ס����
            delay_state ++;
            if(delay_state > 100)
            {
                run_state = 1;//�������ϸ�λ
                delay_state = 0;
            }                  
        }              
    }
    else if(run_state == 1)//�������ϸ�λ
    {
        if(get_pos < 0)
        {
            if(get_pos > -150000)
            {
                set_speed = 800;
                pid_calc_flag = 1;
            }
            else
            {
                current_set_value = 2500;
                pid_calc_flag = 2;
            }
        }
        else if(get_pos >= 0)
        {
            pid_calc_flag = 1;
            set_speed = 0;
            run_state = 2;//������ת�����־λ��λ
            storage_times ++;
            storage_state = 1;//�������
        }
    }        
  }
}


/**
  * @brief				�ͷ�
  * @param[in]		
	* @param[out]		
  * @retval				
*/
void Energy_Release(void)
{
  if(storage_state == 1)
  {
    Shoot_Firction_Motor(150,150);//�����
    storage_state = 0;
  }
  release_delay_state++;
}
/**
  * @brief				PID��ʼ��������3508��
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void EnergyStorage_Motor_PID_Init(void)
{
	PID_Param_Init(&motor_pid[PID_CHASSIS_MOTOR_RF_SPD], POSITION_PID, 15000, 11000,
									15.0f, 0.1f, 0.0f);
}

//===================================================================================================================//
/*******************************************************����**********************************************************/
//===================================================================================================================//
/**
  * @brief				��������(����������)
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Trigger_Drive_Task(void const * argument)
{ 
  static int32_t trigger_delay_time1 = 0;//�����תʱ��
  static int32_t trigger_delay_time2 = 0;//�����תʱ��
  static int8_t reverse_flag = 1;//��ת��־λ
  static int8_t trigger_time = 1;//��ת���� 
  
  Trigger_Motor_PID_Init();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();  
	
	for(;;)
	{
    Refresh_Task_OffLine_Time(TriggerDriveTask_TOE);//��¼�������е�ʱ���
    shoot_control.trigger_get_pos = motor_get[TRIGGER_MOTOR].total_angle;
    shoot_control.trigger_get_spd = motor_get[TRIGGER_MOTOR].speed_rpm;  

      if(shoot_control.storage_init_flag == 1)
      {
        if(trigger_time == 1)
        {
            if(storage_times == trigger_time)
            {
               if(reverse_flag == 1)
               {
                 HAL_GPIO_WritePin(POWER1_GPIO_Port,POWER1_Pin,GPIO_PIN_SET);
                 shoot_control.trigger_set_spd = -400;
                 trigger_delay_time2++;
               }
               if(trigger_delay_time2 > 80)
               {
                  reverse_flag = 0;
                  trigger_delay_time2 = 0;
               }
               if(reverse_flag == 0)
               {             
                  shoot_control.trigger_set_spd = 400;
                  trigger_delay_time1++;
                  if(trigger_delay_time1 > 400)
                  {
                    HAL_GPIO_WritePin(POWER1_GPIO_Port,POWER1_Pin,GPIO_PIN_RESET);
                    trigger_time++;
                    reverse_flag = 1;
                    trigger_delay_time1 = 0;
                  } 
               }              
            }
        }
        else
        {
           if(run_state == 1  && get_pos > -500000 && (shoot_control.rotate_times + 1) == trigger_time)
           {
               if(reverse_flag == 1)
               {
                 HAL_GPIO_WritePin(POWER1_GPIO_Port,POWER1_Pin,GPIO_PIN_SET);
                 shoot_control.trigger_set_spd = -400;
                 trigger_delay_time2++;
               }
               if(trigger_delay_time2 > 80)
               {
                  reverse_flag = 0;
                  trigger_delay_time2 = 0;
               }
               if(reverse_flag == 0)
               {             
                  shoot_control.trigger_set_spd = 400;
                  trigger_delay_time1++;
                  if(trigger_delay_time1 > 400)
                  {
                    HAL_GPIO_WritePin(POWER1_GPIO_Port,POWER1_Pin,GPIO_PIN_RESET);
                    trigger_time++;
                    reverse_flag = 1;
                    trigger_delay_time1 = 0;
                  } 
               }              
           }                    
        }

      }
     shoot_control.trigger_current_value = 
        PID_Calc(&motor_pid[PID_TRIGGER_MOTOR_SPD], shoot_control.trigger_get_spd, shoot_control.trigger_set_spd); 
    
    Gimbal_Motor_Drive(&hcan1,0,0,shoot_control.trigger_current_value);
    

		osDelayUntil(&xLastWakeTime,TRIGGER_PERIOD);		
    
	}

}

/**
  * @brief				PID��ʼ��������3508��
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Trigger_Motor_PID_Init(void)
{     
 	PID_Param_Init(&motor_pid[PID_TRIGGER_MOTOR_SPD], POSITION_PID, 3000, 2000,
									10.0f, 0.3f, 0.0f);
}


