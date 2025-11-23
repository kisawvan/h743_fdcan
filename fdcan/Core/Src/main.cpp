/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
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
#include "stm32h743xx.h"
#include "main.h"
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "matrix.h"
#include "robotics.h"
#include <stdio.h>
#include <cmath>
#include "arm_math.h"

// 禁用半主机模式
__asm (".global __use_no_semihosting\n");

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t heartbeat_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
// 只声明必要的函数，不要重复声明rt_sys.h中已有的函数
extern "C" {
    int fputc(int ch, FILE *f);
    int fgetc(FILE *f);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 声明外部串口句柄
extern UART_HandleTypeDef huart1;

// 系统调用实现 - 只实现必要的函数
extern "C" {
    
    // 必须的函数
    void _sys_exit(int return_code) {
        while(1) {}
    }

    void _ttywrch(int ch) {
        uint8_t byte = (uint8_t)(ch & 0xFF);
        HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
    }

    // printf重定向
    int fputc(int ch, FILE *f) {
        uint8_t byte = (uint8_t)(ch & 0xFF);
        HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
        return ch;
    }

    int fgetc(FILE *f) {
        uint8_t byte;
        HAL_UART_Receive(&huart1, &byte, 1, HAL_MAX_DELAY);
        return (int)byte;
    }

    // 可选：使用_write和_read替代fputc/fgetc
    int _write(int file, char *ptr, int len) {
        if (file == 1 || file == 2) {  // stdout or stderr
            HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
            return len;
        }
        return -1;
    }

    int _read(int file, char *ptr, int len) {
        if (file == 0 && len > 0) {  // stdin
            HAL_UART_Receive(&huart1, (uint8_t*)ptr, 1, HAL_MAX_DELAY);
            return 1;
        }
        return -1;
    }
}

// FDCAN接收回调函数实现
void FDCAN1_ReceiveCallback(uint32_t id, uint8_t *data, uint8_t length)
{
    printf("Received CAN Message - ID: 0x%03X, Length: %d, Data: ", id, length);
    for(int i = 0; i < length; i++) {
        printf("%02X ", data[i]);
    }
    printf("\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init(); 
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0xffff);
  
  printf("System initialized, starting main loop...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    uint8_t heartbeat_data[4] = {0x00,0x00,0x00,0x00};
    HAL_StatusTypeDef status = FDCAN1_SendMessage(0x01, heartbeat_data, 4);
    
    // 示例2：发送机器人状态数据
    uint8_t robot_status[8] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};
    status = FDCAN1_SendMessage(0x200, robot_status, 8);
    if (status == HAL_OK) {
        printf("Robot status sent - ID: 0x200\r\n");
    }
    uint8_t test_data[2] = {0xDE, 0xAD};
    status = FDCAN1_SendMessage(0x001, test_data, 2);
    if (status == HAL_OK) {
        printf("Test data sent - ID: 0x001\r\n");
    }

    float m[6] = {0.2645f, 0.17f, 0.1705f, 0, 0, 0};      /* 各连杆质量 */
    
    /* 各连杆质心坐标（3x6矩阵）*/
    Matrixf<3, 6> rc((float[18]){0, -8.5e-2f, 0, 0, 0, 0, 
                                 13.225e-2f, 0, 0, 0, 0, 0,
                                 0, 3.7e-2f, 8.525e-2f, 0, 0, 0});
    
    Matrixf<3, 3> I[6];            /* 定义各连杆转动惯量矩阵（3x3）*/
    I[0] = matrixf::diag<3, 3>((float[3]){1.542e-3f, 0, 1.542e-3f});
    I[1] = matrixf::diag<3, 3>((float[3]){0, 0.409e-3f, 0.409e-3f});
    I[2] = matrixf::diag<3, 3>((float[3]){0.413e-3f, 0.413e-3f, 0});
    I[3] = matrixf::eye<3, 3>() * 3.0f;      
    I[4] = matrixf::eye<3, 3>() * 2.0f;
    I[5] = matrixf::eye<3, 3>() * 1.0f;

    robotics::Link links[6];          /* 定义六连杆数组*/
    links[0] = robotics::Link(0, 26.45e-2f, 0, -M_PI / 2, robotics::R, 0, 0, 0, m[0], rc.col(0), I[0]);
    links[1] = robotics::Link(0, 5.5e-2f, 17e-2f, 0, robotics::R, 0, 0, 0, m[1], rc.col(1), I[1]);
    links[2] = robotics::Link(0, 0, 0, -M_PI / 2, robotics::R, 0, 0, 0, m[2], rc.col(2), I[2]);
    links[3] = robotics::Link(0, 17.05e-2f, 0, M_PI / 2, robotics::R, 0, 0, 0, m[3], rc.col(3), I[3]);
    links[4] = robotics::Link(0, 0, 0, -M_PI / 2, robotics::R, 0, 0, 0, m[4], rc.col(4), I[4]);
    links[5] = robotics::Link(0, 0, 0, 0, robotics::R, 0, 0, 0, m[5], rc.col(5), I[5]);
    
    robotics::Serial_Link<6> p560(links);    /* 定义6DOF机器人*/
    
    /* 定义关节变量q、速度qv、加速度qa、末端所受外力he*/
    float q[6] = {0.2f, -0.5f, -0.3f, -0.6f, 0.5f, 0.2f};   /* 关节角度 */
    float qv[6] = {1.0f, 0.5f, -1.0f, 0.3f, 0.0f, -1.0f};   /* 关节速度 */
    float qa[6] = {0.2f, -0.3f, 0.1f, 0.0f, -1.0f, 0.0f};   /* 关节加速度 */
    float he[6] = {1.0f, 2.0f, -3.0f, -0.5f, -2.0f, 1.0f};  /* 末端外力 */

    // 执行机器人计算
    Matrixf<4, 4> T = p560.fkine(q);                  /* 正运动学 */
    Matrixf<6, 6> J = p560.jacob(q);                  /* 雅可比矩阵 */
    Matrixf<6, 1> q_ikine = p560.ikine(T, Matrixf<6, 1>((float[6]){0, 0, 0, 0, 0.1f, 0})); /* 逆运动学 */
    Matrixf<6, 1> torq = p560.rne(q, qv, qa, he);     /* 逆动力学 */
    
    printf("Robot calculation completed.\r\n");
    
    HAL_Delay(2000);  // 2秒延迟
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 10;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif
