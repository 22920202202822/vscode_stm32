/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
#define m_CAN1 0
#define m_CAN2 1
typedef struct CAN_s_Rx_Temp_t
{
  CAN_RxHeaderTypeDef can_rx_temp;
  uint8_t can_rx_array[8];
}CAN_s_Rx_Temp;
typedef struct CAN_s_Tx_Temp_t
{
  CAN_TxHeaderTypeDef can_tx_temp;
  uint8_t can_tx_array[8];
  uint32_t mailbox;
}CAN_s_Tx_Temp;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void MX_CAN1_Filter_Config(void);
void MX_CAN2_Filter_Config(void);
void CAN_Send_Msg(uint8_t CAN_x, uint32_t Std_ID, uint8_t* pdata, uint8_t len);
void CAN_Send_Msg_Init(uint8_t CAN_x);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

