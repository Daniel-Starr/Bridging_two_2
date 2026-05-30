/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32u5xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
DMA_HandleTypeDef handle_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel4;
DMA_HandleTypeDef handle_GPDMA1_Channel5;   /* FIX: USART1 TX DMA(PC 端口非阻塞发送) */
DMA_HandleTypeDef handle_GPDMA1_Channel6;   /* FIX: UART4 TX DMA */
DMA_HandleTypeDef handle_GPDMA1_Channel7;   /* FIX: LPUART1 TX DMA */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void MX_ConfigUartRxDma(UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef *hdma,
                               DMA_Channel_TypeDef *channel,
                               uint32_t request,
                               IRQn_Type irq);
static void MX_ConfigUartTxDma(UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef *hdma,
                               DMA_Channel_TypeDef *channel,
                               uint32_t request,
                               IRQn_Type irq);

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(huart->Instance==LPUART1)
  {
    /* USER CODE BEGIN LPUART1_MspInit 0 */

    /* USER CODE END LPUART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_LPUART1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**LPUART1 GPIO Configuration
    PC0     ------> LPUART1_RX
    PC1     ------> LPUART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* LPUART1 interrupt Init */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
    /* USER CODE BEGIN LPUART1_MspInit 1 */
    MX_ConfigUartRxDma(huart,
                       &handle_GPDMA1_Channel0,
                       GPDMA1_Channel0,
                       GPDMA1_REQUEST_LPUART1_RX,
                       GPDMA1_Channel0_IRQn);
    MX_ConfigUartTxDma(huart,
                       &handle_GPDMA1_Channel7,
                       GPDMA1_Channel7,
                       GPDMA1_REQUEST_LPUART1_TX,
                       GPDMA1_Channel7_IRQn);

    /* USER CODE END LPUART1_MspInit 1 */
  }
  else if(huart->Instance==UART4)
  {
    /* USER CODE BEGIN UART4_MspInit 0 */

    /* USER CODE END UART4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    /* USER CODE BEGIN UART4_MspInit 1 */
    MX_ConfigUartRxDma(huart,
                       &handle_GPDMA1_Channel1,
                       GPDMA1_Channel1,
                       GPDMA1_REQUEST_UART4_RX,
                       GPDMA1_Channel1_IRQn);
    MX_ConfigUartTxDma(huart,
                       &handle_GPDMA1_Channel6,
                       GPDMA1_Channel6,
                       GPDMA1_REQUEST_UART4_TX,
                       GPDMA1_Channel6_IRQn);

    /* USER CODE END UART4_MspInit 1 */
  }
  else if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */

    /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspInit 1 */
    MX_ConfigUartRxDma(huart,
                       &handle_GPDMA1_Channel2,
                       GPDMA1_Channel2,
                       GPDMA1_REQUEST_USART1_RX,
                       GPDMA1_Channel2_IRQn);
    MX_ConfigUartTxDma(huart,
                       &handle_GPDMA1_Channel5,
                       GPDMA1_Channel5,
                       GPDMA1_REQUEST_USART1_TX,
                       GPDMA1_Channel5_IRQn);

    /* USER CODE END USART1_MspInit 1 */
  }
  else if(huart->Instance==USART3)
  {
    /* USER CODE BEGIN USART3_MspInit 0 */

    /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PA5     ------> USART3_RX
    PA7     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    /* USER CODE BEGIN USART3_MspInit 1 */
    MX_ConfigUartRxDma(huart,
                       &handle_GPDMA1_Channel3,
                       GPDMA1_Channel3,
                       GPDMA1_REQUEST_USART3_RX,
                       GPDMA1_Channel3_IRQn);
    MX_ConfigUartTxDma(huart,
                       &handle_GPDMA1_Channel4,
                       GPDMA1_Channel4,
                       GPDMA1_REQUEST_USART3_TX,
                       GPDMA1_Channel4_IRQn);

    /* USER CODE END USART3_MspInit 1 */
  }

}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==LPUART1)
  {
    /* USER CODE BEGIN LPUART1_MspDeInit 0 */

    /* USER CODE END LPUART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPUART1_CLK_DISABLE();

    /**LPUART1 GPIO Configuration
    PC0     ------> LPUART1_RX
    PC1     ------> LPUART1_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1);

    /* LPUART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(LPUART1_IRQn);
    /* USER CODE BEGIN LPUART1_MspDeInit 1 */
    if (huart->hdmarx != 0)
    {
      HAL_DMA_DeInit(huart->hdmarx);
    }
    HAL_NVIC_DisableIRQ(GPDMA1_Channel0_IRQn);

    /* USER CODE END LPUART1_MspDeInit 1 */
  }
  else if(huart->Instance==UART4)
  {
    /* USER CODE BEGIN UART4_MspDeInit 0 */

    /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    /* UART4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    /* USER CODE BEGIN UART4_MspDeInit 1 */
    if (huart->hdmarx != 0)
    {
      HAL_DMA_DeInit(huart->hdmarx);
    }
    HAL_NVIC_DisableIRQ(GPDMA1_Channel1_IRQn);

    /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */

    /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspDeInit 1 */
    if (huart->hdmarx != 0)
    {
      HAL_DMA_DeInit(huart->hdmarx);
    }
    HAL_NVIC_DisableIRQ(GPDMA1_Channel2_IRQn);

    /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(huart->Instance==USART3)
  {
    /* USER CODE BEGIN USART3_MspDeInit 0 */

    /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PA5     ------> USART3_RX
    PA7     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_7);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    /* USER CODE BEGIN USART3_MspDeInit 1 */
    if (huart->hdmarx != 0)
    {
      HAL_DMA_DeInit(huart->hdmarx);
    }
    HAL_NVIC_DisableIRQ(GPDMA1_Channel3_IRQn);
    if (huart->hdmatx != 0)
    {
      HAL_DMA_DeInit(huart->hdmatx);
    }
    HAL_NVIC_DisableIRQ(GPDMA1_Channel4_IRQn);

    /* USER CODE END USART3_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */
static void MX_ConfigUartRxDma(UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef *hdma,
                               DMA_Channel_TypeDef *channel,
                               uint32_t request,
                               IRQn_Type irq)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  hdma->Instance = channel;
  hdma->Init.Request = request;
  hdma->Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma->Init.SrcInc = DMA_SINC_FIXED;
  hdma->Init.DestInc = DMA_DINC_INCREMENTED;
  hdma->Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma->Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  /* FIX: USART3 RX 用高 DMA 优先级,三个 PC 端口保持低优先级 */
  hdma->Init.Priority = (request == GPDMA1_REQUEST_USART3_RX) ? DMA_HIGH_PRIORITY : DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma->Init.SrcBurstLength = 1U;
  hdma->Init.DestBurstLength = 1U;
  hdma->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma->Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma->Init.Mode = DMA_NORMAL;

  if (HAL_DMA_Init(hdma) != HAL_OK)
  {
    Error_Handler();
  }

  huart->hdmarx = hdma;
  hdma->Parent = huart;

  HAL_NVIC_SetPriority(irq, 0, 0);
  HAL_NVIC_EnableIRQ(irq);
}

static void MX_ConfigUartTxDma(UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef *hdma,
                               DMA_Channel_TypeDef *channel,
                               uint32_t request,
                               IRQn_Type irq)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  hdma->Instance = channel;
  hdma->Init.Request = request;
  hdma->Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma->Init.SrcInc = DMA_SINC_INCREMENTED;
  hdma->Init.DestInc = DMA_DINC_FIXED;
  hdma->Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma->Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  /* FIX: USART3 TX 用高 DMA 优先级,PC 端口 TX(批次2新增)保持低优先级 */
  hdma->Init.Priority = (request == GPDMA1_REQUEST_USART3_TX) ? DMA_HIGH_PRIORITY : DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma->Init.SrcBurstLength = 1U;
  hdma->Init.DestBurstLength = 1U;
  hdma->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma->Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma->Init.Mode = DMA_NORMAL;

  if (HAL_DMA_Init(hdma) != HAL_OK)
  {
    Error_Handler();
  }

  huart->hdmatx = hdma;
  hdma->Parent = huart;

  HAL_NVIC_SetPriority(irq, 0, 0);
  HAL_NVIC_EnableIRQ(irq);
}

/* USER CODE END 1 */
