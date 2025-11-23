/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "string.h"

// 全局变量定义
FDCAN_HandleTypeDef FDCAN1_Handler;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;

// 收发缓冲区
uint8_t FDCAN1_RxData[64];  // FD模式支持最大64字节
uint8_t FDCAN1_TxData[64];  // FD模式支持最大64字节

// 心跳计数器
static uint32_t heartbeat_counter = 0;
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{
  /* USER CODE BEGIN FDCAN1_Init 0 */
  FDCAN_FilterTypeDef MX_FDCAN1_RXFilter;
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;    // FD模式，无比特率切换
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;               // 正常模式
  hfdcan1.Init.AutoRetransmission = DISABLE;           // 必须为DISABLE
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  
  // 仲裁阶段参数（500Kbps @ 20MHz时钟）
  hfdcan1.Init.NominalPrescaler = 2;                   // 分频系数
  hfdcan1.Init.NominalSyncJumpWidth = 1;               // 同步跳转宽度
  hfdcan1.Init.NominalTimeSeg1 = 14;                   // 时间段1
  hfdcan1.Init.NominalTimeSeg2 = 5;                    // 时间段2
  
  // 数据阶段参数（与仲裁阶段相同速率，因为使用NO_BRS）
  hfdcan1.Init.DataPrescaler = 2;                      // 数据阶段分频
  hfdcan1.Init.DataSyncJumpWidth = 1;                  // 数据阶段同步跳转宽度
  hfdcan1.Init.DataTimeSeg1 = 14;                      // 数据阶段时间段1
  hfdcan1.Init.DataTimeSeg2 = 5;                       // 数据阶段时间段2
  
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;                      // 标准滤波器数量
  hfdcan1.Init.ExtFiltersNbr = 0;                      // 扩展滤波器数量
  hfdcan1.Init.RxFifo0ElmtsNbr = 4;                    // RX FIFO0元素数量
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;  // FD模式支持最大64字节
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 3;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;       // FD模式支持最大64字节
  
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  // 初始化全局句柄
  FDCAN1_Handler = hfdcan1;
  
  // 配置RX滤波器  
  MX_FDCAN1_RXFilter.IdType = FDCAN_STANDARD_ID;                       
  MX_FDCAN1_RXFilter.FilterIndex = 0;                                                    
  MX_FDCAN1_RXFilter.FilterType = FDCAN_FILTER_MASK;                  
  MX_FDCAN1_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           
  MX_FDCAN1_RXFilter.FilterID1 = 0x0000;                               
  MX_FDCAN1_RXFilter.FilterID2 = 0x0000;                              
  if (HAL_FDCAN_ConfigFilter(&FDCAN1_Handler, &MX_FDCAN1_RXFilter) != HAL_OK)
  {
    Error_Handler();
  }
  
  // 启动FDCAN                       
  if (HAL_FDCAN_Start(&FDCAN1_Handler) != HAL_OK)
  {
    Error_Handler();
  }
  
  // 激活接收中断
  if (HAL_FDCAN_ActivateNotification(&FDCAN1_Handler, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
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

/**
  * @brief  FDCAN发送报文函数（FD模式）
  * @param  id: 报文ID
  * @param  data: 发送数据指针
  * @param  length: 数据长度(0-64)
  * @retval HAL status
  */
HAL_StatusTypeDef FDCAN1_SendMessage(uint32_t id, uint8_t *data, uint8_t length)
{
    // 配置发送报文头 - FD模式配置
    FDCAN1_TxHeader.Identifier = id;
    FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;
    FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN1_TxHeader.DataLength = (uint32_t)length << 16;  // 数据长度左移16位
    
    // FD模式特有配置
    FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        // 无比特率切换（与NO_BRS对应）
    FDCAN1_TxHeader.FDFormat = FDCAN_FD_CAN;              // FD帧格式
    FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCAN1_TxHeader.MessageMarker = 0;
    
    // 复制数据到发送缓冲区（FD模式最多64字节）
    if (data != NULL && length > 0 && length <= 64)
    {
        memcpy(FDCAN1_TxData, data, length);
    }
    else if (length == 0)
    {
        // 无数据帧，清空发送缓冲区
        memset(FDCAN1_TxData, 0, sizeof(FDCAN1_TxData));
    }
    
    // 发送报文
    return HAL_FDCAN_AddMessageToTxFifoQ(&FDCAN1_Handler, &FDCAN1_TxHeader, FDCAN1_TxData);
}

/**
  * @brief  发送心跳帧示例函数
  * @retval None
  */
void FDCAN1_SendHeartbeat(void)
{
    uint8_t heartbeat_data[8];
    
    // 填充心跳数据
    heartbeat_data[0] = 0xAA;  // 同步头
    heartbeat_data[1] = 0x55;  // 同步尾
    heartbeat_data[2] = (heartbeat_counter >> 24) & 0xFF;  // 计数器高位
    heartbeat_data[3] = (heartbeat_counter >> 16) & 0xFF;
    heartbeat_data[4] = (heartbeat_counter >> 8) & 0xFF;
    heartbeat_data[5] = heartbeat_counter & 0xFF;          // 计数器低位
    heartbeat_data[6] = 0x01;  // 设备状态
    heartbeat_data[7] = 0x00;  // 保留
    
    // 发送心跳帧，ID = 0x100
    HAL_StatusTypeDef status = FDCAN1_SendMessage(0x100, heartbeat_data, 8);
    
    if (status == HAL_OK)
    {
        heartbeat_counter++;
    }
}

/**
  * @brief  FDCAN接收中断回调函数
  * @param  hfdcan: FDCAN句柄指针
  * @param  RxFifo0ITs: 接收FIFO0中断标志
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        // 从FIFO0读取报文
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxHeader, FDCAN1_RxData) == HAL_OK)
        {
            // 计算数据长度
            uint8_t data_length = FDCAN1_RxHeader.DataLength >> 16;
            
            // 调用用户数据处理函数
            FDCAN1_ReceiveCallback(FDCAN1_RxHeader.Identifier, FDCAN1_RxData, data_length);
        }
    }
}

/**
  * @brief  用户自定义的接收数据处理回调函数（需要用户实现）
  * @param  id: 接收到的报文ID
  * @param  data: 接收到的数据指针
  * @param  length: 数据长度
  * @retval None
  */
__weak void FDCAN1_ReceiveCallback(uint32_t id, uint8_t *data, uint8_t length)
{
    // 用户需要在自己的代码中重写这个函数来处理接收到的数据
    // 示例：打印接收到的信息
    // printf("Received FD CAN Message - ID: 0x%03X, Length: %d\n", id, length);
    
    // 防止未使用参数警告
    (void)id;
    (void)data;
    (void)length;
}

/**
  * @brief  检查FDCAN是否准备好发送
  * @retval HAL status
  */
HAL_StatusTypeDef FDCAN1_CheckReadyToSend(void)
{
    return (FDCAN1_Handler.Instance->TXFQS & FDCAN_TXFQS_TFQF) ? HAL_ERROR : HAL_OK;
}

/**
  * @brief  获取FDCAN发送FIFO剩余空间
  * @retval 剩余空间数量
  */
uint32_t FDCAN1_GetTxFifoFreeLevel(void)
{
    return 3 - ((FDCAN1_Handler.Instance->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);
}

/**
  * @brief  获取FDCAN接收FIFO0中的数据数量
  * @retval 数据数量
  */
uint32_t FDCAN1_GetRxFifo0Level(void)
{
    return (FDCAN1_Handler.Instance->RXF0S & FDCAN_RXF0S_F0FL) >> FDCAN_RXF0S_F0FL_Pos;
}

/**
  * @brief  检查FDCAN错误状态
  * @retval None
  */

/* USER CODE END 1 */
