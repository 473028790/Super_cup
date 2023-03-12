/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "ina219.h"
#include "dac.h"
#include "adc.h"
#include "fdcan.h"
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
uint16_t AD_Value;
float AD_Value1;
extern uint32_t adc_value[1];
float Vcc_Battery;
float Vcc_Output;

float dac_out=0;

extern struct supercap cap;

/* USER CODE END Variables */
osThreadId thread_TransferHandle;
osThreadId Vcc_TaskHandle;
osThreadId ControlHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Thread_Transfer(void const * argument);
void Thread_Vcc(void const * argument);
void Thread_Control(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* definition and creation of thread_Transfer */
  osThreadDef(thread_Transfer, Thread_Transfer, osPriorityHigh, 0, 128);
  thread_TransferHandle = osThreadCreate(osThread(thread_Transfer), NULL);

  /* definition and creation of Vcc_Task */
  osThreadDef(Vcc_Task, Thread_Vcc, osPriorityHigh, 0, 128);
  Vcc_TaskHandle = osThreadCreate(osThread(Vcc_Task), NULL);

  /* definition and creation of Control */
  osThreadDef(Control, Thread_Control, osPriorityHigh, 0, 128);
  ControlHandle = osThreadCreate(osThread(Control), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Thread_Transfer */
/**
  * @brief  Function implementing the thread_Transfer thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Thread_Transfer */
void Thread_Transfer(void const * argument)
{
  /* USER CODE BEGIN Thread_Transfer */
  /* Infinite loop */
  for(;;)
  {
		CAN1_0x1FF_TX(INA219_D.Voltage,INA219_D.Power,Vcc_Battery,Vcc_Battery);
    /*
		Transfer_data[0] = INA219_D.Voltage;
		Transfer_data[1] = INA219_D.Power;
		Transfer_data[2] = 0;
*/
    osDelay(1);
  }
  /* USER CODE END Thread_Transfer */
}

/* USER CODE BEGIN Header_Thread_Vcc */
/**
* @brief Function implementing the Vcc_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_Vcc */
void Thread_Vcc(void const * argument)
{
  /* USER CODE BEGIN Thread_Vcc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Thread_Vcc */
}

/* USER CODE BEGIN Header_Thread_Control */
/**
* @brief Function implementing the Control thread.
* @param argument: Not used
* @retval None
*/
int cnt10=0;
/* USER CODE END Header_Thread_Control */
void Thread_Control(void const * argument)
{
  /* USER CODE BEGIN Thread_Control */
  /* Infinite loop */
  for(;;)
  {
    INA_GET_Voltage_MV();//获取当前电压
		INA_GET_Current_MA();//获取当前电流
		INA_GET_Power_MW();//获取当前功率

		/*
		HAL_ADC_Start(&hadc1);//启动adc转换
		HAL_ADC_PollForConversion(&hadc1, 0);//等待转换完成，第二个参数表示超时时间，单位ms
		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))//
			{
					AD_Value = HAL_ADC_GetValue(&hadc1);//读取ADC转换数据，数据为12位
				AD_Value1=(float)AD_Value*33/4096;
				cnt10++;
			}
		*/
		HAL_ADC_Start_DMA(&hadc1,adc_value,1);//开启ADC的DMA接收
		Vcc_Battery = adc_value[0]*3.3f/4096.0f*10;
			
			
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		
		dac_out=(cap.Bat_V)*0.2/Vcc_Battery;
		if(dac_out>1) dac_out=1;
		dac_out=dac_out/3.3*4095;
    HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,dac_out);
    osDelay(10);
  }
  /* USER CODE END Thread_Control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

