/*******************************************************************************
                      版权所有 (C), 2020-,NCUROBOT
 *******************************************************************************
  文 件 名   : shoot_task.c
  版 本 号   : V1.0
  作    者   : 高云海
  生成日期   : 2021.7.1
  最近修改   :
  功能描述   : 飞镖发射任务，包括蓄能任务与拨盘（飞镖换弹）任务
  函数列表   : 1) Energy_Storage_Task()		【FreeRTOS函数：操作系统任务调度运行】
							 2) Trigger_Drive_Task()		【FreeRTOS函数：操作系统任务调度运行】
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "shoot_task.h"
#include "motor_use_tim.h"
#include "remote_control.h"
#include "remote_control_task.h"
#include "motor_use_can.h"
#include "offline_check.h"
#include "referee.h"
#include "pid.h"

/* 内部宏定义 ----------------------------------------------------------------*/

/* 内部自定义数据类型的变量 --------------------------------------------------*/
shoot_control_t shoot_control;   //发射数据结构体

/* 内部变量 ------------------------------------------------------------------*/
static  int8_t  storage_state = 0;//蓄能完成标志位【0未完成，1已完成】
static  int8_t  pid_calc_flag = 0;//【是否进行PID运算标志位；1为运算；2为不运算】
static  int8_t  storage_times = 0;//蓄能完成次数【初始为0】
static  int8_t  run_state = 2;//链条运转方向【1向上，2向下】
static  int16_t delay_state = 0;//延时标志位【用于待舵机完全落下后链条再开始向上复位】
static  int16_t release_delay_state = 0;//释放后延时标志位【用于完全释放后再进行下一次蓄能】
static  int16_t second_delay_time = 0;//连续两发飞镖中间延长时间
static  int8_t  get_dart_cmd_flag1 = 1;//获取裁判系统飞镖第一发指令标志位
static  int8_t  get_dart_cmd_flag2 = 1;//获取裁判系统飞镖第三发指令标志位
static  float   current_set_value = 0;
static  int16_t get_speed;
static  int16_t set_speed;
static  int32_t get_pos;

/* 内部函数原型声明 ----------------------------------------------------------*/
void Trigger_Motor_PID_Init(void);
void EnergyStorage_Motor_PID_Init(void);
void Energy_Storage_First(void);
void Energy_Storage(void); 
void Energy_Release(void);


/* 函数主体部分 --------------------------------------------------------------*/
//===================================================================================================================//
/******************************************************摩擦轮*********************************************************/
//===================================================================================================================//
/**
  * @brief				蓄能任务
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
		Refresh_Task_OffLine_Time(FrictionDriveTask_TOE);//记录任务运行的时间点
    get_pos = motor_get[CHASSIS_MOTOR_RF].total_angle;    
		get_speed = motor_get[CHASSIS_MOTOR_RF].speed_rpm;
 
    
  if(shoot_control.storage_init_flag == 0)
  {  
    //模式选择
    if(rc_ctrl.rc.s2 == RIGHT_S2_UP)//遥控控制模式
    {
      shoot_control.dart_control_mode = REMOTE_CONTROL;
      Energy_Storage_First();
    }
    else if(rc_ctrl.rc.s2 == RIGHT_S2_DOWN)//机器人交互控制
    {
      shoot_control.dart_control_mode = ROBOT_INTERACTIVE_CONTROL;
      Energy_Storage_First();
    }  
  }
  else if(shoot_control.storage_init_flag == 1)
  {
    //遥控控制模式
    if(shoot_control.dart_control_mode == REMOTE_CONTROL) 
    {
      shoot_control.rotate_times = remote_control.s1_times_value;
      if(shoot_control.rotate_times == storage_times)
      {
        Energy_Release();
        release_delay_state++;
        if(release_delay_state >= 300 && storage_times < 4)//等待完全释放进行下一次蓄能
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
    //机器人交互控制
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
        if(release_delay_state >= 300 && storage_times < 4)//等待完全释放进行下一次蓄能
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
    
    //PID运算
    if(pid_calc_flag == 1)   
    {  
      current_set_value = PID_Calc(&motor_pid[PID_CHASSIS_MOTOR_RF_SPD], get_speed, set_speed);
    }
    
		Chassis_Motor_Drive(&hcan1,current_set_value,0,0,0);
   
		osDelayUntil(&xLastWakeTime,Energy_PERIOD);		
	}
}

/**
  * @brief				初始蓄能：第一发飞镖的蓄能
  * @param[in]		
	* @param[out]		
  * @retval				
*/
void Energy_Storage_First(void) 
{
  if(storage_state == 0)//蓄能未完成
  {    
    if(run_state == 2)//链条向下蓄能
    {
      if(get_pos >  -1110000)
      {
          Shoot_Firction_Motor(150,150);//舵机打开
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
          Shoot_Firction_Motor(120,120);//舵机关闭
          pid_calc_flag = 1;
          set_speed = 0;//锁住链条
          delay_state ++;
          if(delay_state > 100)
          {
              run_state = 1;//链条向上复位
              delay_state = 0;
          }                  
      }              
    }
    else if(run_state == 1)//链条向上复位
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
          run_state = 2;//链条运转方向标志位复位
          shoot_control.storage_init_flag = 1;
          storage_times++;
          storage_state = 1;//蓄能完成            
      }
    }        
  }
}
/**
  * @brief				后续蓄能：后续飞镖的蓄能
  * @param[in]		
	* @param[out]		
  * @retval				
*/
void Energy_Storage(void) 
{    
  if(storage_state == 0)//蓄能未完成
  {
    if(run_state == 2)//链条向下蓄能
    {
        if(get_pos >  -1110000)
        {
            Shoot_Firction_Motor(150,150);//舵机打开
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
            Shoot_Firction_Motor(120,120);//舵机关闭
            pid_calc_flag = 1;
            set_speed = 0;//锁住链条
            delay_state ++;
            if(delay_state > 100)
            {
                run_state = 1;//链条向上复位
                delay_state = 0;
            }                  
        }              
    }
    else if(run_state == 1)//链条向上复位
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
            run_state = 2;//链条运转方向标志位复位
            storage_times ++;
            storage_state = 1;//蓄能完成
        }
    }        
  }
}


/**
  * @brief				释放
  * @param[in]		
	* @param[out]		
  * @retval				
*/
void Energy_Release(void)
{
  if(storage_state == 1)
  {
    Shoot_Firction_Motor(150,150);//舵机打开
    storage_state = 0;
  }
  release_delay_state++;
}
/**
  * @brief				PID初始化函数（3508）
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
/*******************************************************拨盘**********************************************************/
//===================================================================================================================//
/**
  * @brief				拨盘任务(换飞镖任务)
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Trigger_Drive_Task(void const * argument)
{ 
  static int32_t trigger_delay_time1 = 0;//电机正转时间
  static int32_t trigger_delay_time2 = 0;//电机反转时间
  static int8_t reverse_flag = 1;//反转标志位
  static int8_t trigger_time = 1;//旋转次数 
  
  Trigger_Motor_PID_Init();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();  
	
	for(;;)
	{
    Refresh_Task_OffLine_Time(TriggerDriveTask_TOE);//记录任务运行的时间点
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
  * @brief				PID初始化函数（3508）
  * @param[in]		
	* @param[out]		
  * @retval				none
*/
void Trigger_Motor_PID_Init(void)
{     
 	PID_Param_Init(&motor_pid[PID_TRIGGER_MOTOR_SPD], POSITION_PID, 3000, 2000,
									10.0f, 0.3f, 0.0f);
}


