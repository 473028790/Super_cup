/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */

struct supercap cap;
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 5;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 17;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  sFilterConfig.IdType = FDCAN_STANDARD_ID;   //  配置为过滤标准帧
  sFilterConfig.FilterIndex = 0;              //  过滤器的索引号
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;  //  过滤方式为范围，即从FilterID1~FilterID2之间的值
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0000;
  sFilterConfig.FilterID2 = 0x07ff;           //  标准帧为11位ID，即0x7ff，本例配置为接收所有帧
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();
  
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;   //  配置为过滤扩展帧
  sFilterConfig.FilterIndex = 0;              //  过滤器的索引号
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;  //  过滤方式为范围，即从FilterID1~FilterID2之间的值
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x01ffffff;       //  扩展帧为29位ID，即0x1fffffff，本例配置为接收所有帧
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  HAL_FDCAN_Start(&hfdcan1);
}

void CAN1_0x1FF_TX(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	FDCAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.Identifier = 0x666;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 1;
	
	
	static uint8_t Data[8];
	Data[0]=motor1>>8;
	Data[1]=motor1;
	Data[2]=motor2>>8;
	Data[3]=motor2;
	Data[4]=motor3>>8;
	Data[5]=motor3;
	Data[6]=motor4<15?1:2;
	Data[7]=0;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, Data);
	
}


void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
  FDCAN_RxHeaderTypeDef CAN_RX;


    static uint8_t Data[8];
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &CAN_RX, Data);
		switch(CAN_RX.Identifier)
		{
    	case 0x333://supercap
        cap.Bat_V=(float)((int16_t)(Data[0]<<8|Data[1]));
			break; 
			default: {break;}	
		}

}
/* USER CODE END 1 */
