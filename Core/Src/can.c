/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "string.h"
CAN_s_Rx_Temp hcan1_rx_temp;
CAN_s_Rx_Temp hcan2_rx_temp;
CAN_s_Tx_Temp hcan1_tx_temp;
CAN_s_Tx_Temp hcan2_tx_temp;
extern uint8_t can1_data[8][8];
extern uint8_t can2_data[8][8];
uint32_t can1_mailbox = CAN_TX_MAILBOX0;
uint32_t can2_mailbox = CAN_TX_MAILBOX1;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  MX_CAN1_Filter_Config();
  CAN_Send_Msg_Init(m_CAN1);
  if(HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)					// 使能CAN接收中断
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY)!=HAL_OK)					// 使能CAN发�?�中�?
	{
		Error_Handler();
	}
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  MX_CAN2_Filter_Config();
  CAN_Send_Msg_Init(m_CAN2);
  if(HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)					// 使能CAN接收中断
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY)!=HAL_OK)					// 使能CAN发�?�中�?
	{
		Error_Handler();
	}
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void MX_CAN1_Filter_Config(void)
{
  CAN_FilterTypeDef CAN1_FilerConf;	
	CAN1_FilerConf.FilterIdHigh=0X0000;     //32位ID
  CAN1_FilerConf.FilterIdLow=0X0000;
  CAN1_FilerConf.FilterMaskIdHigh=0X0000; //32位MASK
  CAN1_FilerConf.FilterMaskIdLow=0X0000;  
  CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤�?0关联到FIFO0
  CAN1_FilerConf.FilterBank=0;          //过滤�?0
  CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
  CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
  CAN1_FilerConf.FilterActivation=ENABLE; //�?活滤波器0
  CAN1_FilerConf.SlaveStartFilterBank=14;
  if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK)//滤波器初始化
	{
    Error_Handler();
  }
}
void MX_CAN2_Filter_Config(void)
{
  CAN_FilterTypeDef CAN2_FilerConf;	
	CAN2_FilerConf.FilterIdHigh=0X0000;     //32位ID
  CAN2_FilerConf.FilterIdLow=0X0000;
  CAN2_FilerConf.FilterMaskIdHigh=0X0000; //32位MASK
  CAN2_FilerConf.FilterMaskIdLow=0X0000;  
  CAN2_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤�?0关联到FIFO0
  CAN2_FilerConf.FilterBank=0;          //过滤�?0
  CAN2_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
  CAN2_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
  CAN2_FilerConf.FilterActivation=ENABLE; //�?活滤波器0
  CAN2_FilerConf.SlaveStartFilterBank=14;
  if(HAL_CAN_ConfigFilter(&hcan2,&CAN2_FilerConf)!=HAL_OK)//滤波器初始化
	{
    Error_Handler();
  }
}
void CAN_Send_Msg(uint8_t CAN_x, uint32_t Std_ID, uint8_t* pdata, uint8_t len)
{
  if(CAN_x == m_CAN1)
  {
    memcpy(hcan1_tx_temp.can_tx_array, pdata, len);
    hcan1_tx_temp.can_tx_temp.StdId = Std_ID;
    hcan1_tx_temp.can_tx_temp.DLC = len;
    // 不能加后面的判断语句，开启CAN的自动重传即可。加了会死，因为没有时钟同步机制，可能会与电机发送的报文冲突导致发送失败
    HAL_CAN_AddTxMessage(&hcan1, &hcan1_tx_temp.can_tx_temp,  
			hcan1_tx_temp.can_tx_array, &can1_mailbox);
    // if != HAL_OK)
		// {
		// 	Error_Handler();
		// }
  }
  else
  {
    memcpy(hcan2_tx_temp.can_tx_array, pdata, len);
    hcan2_tx_temp.can_tx_temp.StdId = Std_ID;
    hcan2_tx_temp.can_tx_temp.DLC = len;
    HAL_CAN_AddTxMessage(&hcan2, &hcan2_tx_temp.can_tx_temp, 
			hcan2_tx_temp.can_tx_array, &can2_mailbox);
    // if( != HAL_OK)
		// {
		// 	Error_Handler();
		// }
  }
}
void CAN_Send_Msg_Init(uint8_t CAN_x)
{
  if(CAN_x == m_CAN1)
  {
    hcan1_tx_temp.can_tx_temp.IDE = CAN_ID_STD;
    hcan1_tx_temp.can_tx_temp.RTR = CAN_RTR_DATA;
    hcan1_tx_temp.can_tx_temp.TransmitGlobalTime = DISABLE;
  }
  else
  {
    hcan2_tx_temp.can_tx_temp.IDE = CAN_ID_STD;
    hcan2_tx_temp.can_tx_temp.RTR = CAN_RTR_DATA;
    hcan2_tx_temp.can_tx_temp.TransmitGlobalTime = DISABLE;
  }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// CAN数据接收
	if (hcan->Instance == hcan1.Instance)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1_rx_temp.can_rx_temp, hcan1_rx_temp.can_rx_array) == HAL_OK)		// 获得接收到的数据头和数据
		{
			memcpy(can1_data[hcan1_rx_temp.can_rx_temp.StdId - 0x201], hcan1_rx_temp.can_rx_array, sizeof(uint8_t)*8);
		}
	}
  else
  {
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2_rx_temp.can_rx_temp, hcan2_rx_temp.can_rx_array) == HAL_OK)
    {
      memcpy(can2_data[hcan2_rx_temp.can_rx_temp.StdId - 0x201], hcan2_rx_temp.can_rx_array, sizeof(uint8_t)*8);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);						// 再次使能FIFO0接收中断
}

/* USER CODE END 1 */
