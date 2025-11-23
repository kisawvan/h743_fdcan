/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
// 发送函数
HAL_StatusTypeDef FDCAN1_SendMessage(uint32_t id, uint8_t *data, uint8_t length);

// 接收回调函数（用户需要实现）
void FDCAN1_ReceiveCallback(uint32_t id, uint8_t *data, uint8_t length);

// 工具函数
HAL_StatusTypeDef FDCAN1_CheckReadyToSend(void);
uint32_t FDCAN1_GetTxFifoFreeLevel(void);
uint32_t FDCAN1_GetRxFifo0Level(void);

// 发送心跳帧示例函数
void FDCAN1_SendHeartbeat(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

