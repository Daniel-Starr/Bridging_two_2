/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FRAME_SOF0                     0x55U
#define FRAME_SOF1                     0xAAU
#define FRAME_MAX_PAYLOAD              250U
#define UART_RX_DMA_BUFFER_SIZE        512U
#define USART3_RX_QUEUE_SIZE           8192U
#define USART3_TX_FRAME_SIZE           (2U + 1U + 1U + FRAME_MAX_PAYLOAD + 1U)
#define RESPONSE_QUEUE_DEPTH           128U
#define FRAME_PARSE_TIMEOUT_MS         100U
#define USART3_INTER_FRAME_GAP_MS      1U
#define USART3_TX_TIMEOUT_MS           50U

typedef enum
{
  FRAME_WAIT_SOF0 = 0,
  FRAME_WAIT_SOF1,
  FRAME_WAIT_PORT,
  FRAME_WAIT_LEN,
  FRAME_WAIT_PAYLOAD,
  FRAME_WAIT_CHECKSUM
} FrameParseState;

typedef struct
{
  FrameParseState state;
  uint8_t port_id;
  uint8_t len;
  uint8_t index;
  uint8_t checksum;
  uint8_t payload[FRAME_MAX_PAYLOAD];
} FrameParser;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t *rx_buf[2];
  uint16_t rx_buf_size;
  uint8_t active_rx_buf;
} UartRxDmaContext;

typedef struct
{
  uint8_t port_id;
  uint8_t len;
  uint8_t payload[FRAME_MAX_PAYLOAD];
} ResponseItem;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static FrameParser usart3_frame_parser = {0};
static uint8_t usart3_rx_dma_buf[2][UART_RX_DMA_BUFFER_SIZE];
static uint8_t usart3_rx_queue[USART3_RX_QUEUE_SIZE];
static UartRxDmaContext usart3_rx_ctx = {
  &huart3, {usart3_rx_dma_buf[0], usart3_rx_dma_buf[1]},
  UART_RX_DMA_BUFFER_SIZE, 0U
};
static volatile uint16_t usart3_rx_head = 0U;
static volatile uint16_t usart3_rx_tail = 0U;
static volatile uint32_t usart3_rx_overflow_count = 0U;
static volatile uint32_t uart_dma_restart_error_count = 0U;
static uint8_t usart3_tx_dma_buf[USART3_TX_FRAME_SIZE];
static volatile uint8_t usart3_tx_dma_busy = 0U;
static volatile uint32_t usart3_tx_start_tick = 0U;
static volatile uint32_t usart3_tx_done_tick = 0U;
static volatile uint8_t usart3_rx_restart_pending = 0U;
static ResponseItem response_queue[RESPONSE_QUEUE_DEPTH];
static volatile uint8_t resp_head = 0U;
static volatile uint8_t resp_tail = 0U;
static volatile uint32_t response_queue_overflow_count = 0U;
static uint32_t usart3_frame_parser_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_InitEcho(void);
static void App_ProcessEcho(void);
static HAL_StatusTypeDef App_StartRxDma(UartRxDmaContext *ctx);
static void App_HandleFrame(uint8_t port_id, const uint8_t *payload, uint8_t len);
static void App_TrySendNextResponse(void);
static void App_CheckTxTimeout(void);
static uint16_t App_BuildResponseFrame(uint8_t *buf, uint8_t port_id, const uint8_t *payload, uint8_t len);
static void App_QueueUsart3RxBytes(const uint8_t *payload, uint16_t len);
static uint8_t App_DequeueUsart3RxByte(uint8_t *data);
static void App_RestoreIrqState(uint32_t primask);
static uint8_t FrameParser_Input(FrameParser *parser, uint8_t data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_LPUART1_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  App_InitEcho();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_ProcessEcho();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK)   /* FIX: 开启 USART3 FIFO,提高高速桥接链路抗中断延迟的余量 */
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void App_InitEcho(void)
{
  if (App_StartRxDma(&usart3_rx_ctx) != HAL_OK)
  {
    Error_Handler();
  }
}

static void App_ProcessEcho(void)
{
  uint8_t data;

  if (usart3_rx_restart_pending != 0U)
  {
    if (App_StartRxDma(&usart3_rx_ctx) == HAL_OK)
    {
      usart3_rx_restart_pending = 0U;
    }
  }

  while (App_DequeueUsart3RxByte(&data) != 0U)
  {
    if (FrameParser_Input(&usart3_frame_parser, data) != 0U)
    {
      App_HandleFrame(usart3_frame_parser.port_id,
                      usart3_frame_parser.payload,
                      usart3_frame_parser.len);
    }
  }

  App_CheckTxTimeout();

  /* FIX: 帧解析超时保护——半截帧后断流时强制复位状态机,避免吃掉下一帧的帧头 */
  if (usart3_frame_parser.state != FRAME_WAIT_SOF0)
  {
    if ((HAL_GetTick() - usart3_frame_parser_tick) > FRAME_PARSE_TIMEOUT_MS)
    {
      usart3_frame_parser.state = FRAME_WAIT_SOF0;
    }
  }
  else
  {
    usart3_frame_parser_tick = HAL_GetTick();
  }

  App_TrySendNextResponse();
}

static HAL_StatusTypeDef App_StartRxDma(UartRxDmaContext *ctx)
{
  HAL_StatusTypeDef status;

  if ((ctx == 0) || (ctx->huart == 0) || (ctx->huart->hdmarx == 0))
  {
    return HAL_ERROR;
  }

  __HAL_UART_CLEAR_FLAG(ctx->huart,
                        UART_CLEAR_OREF | UART_CLEAR_FEF |
                        UART_CLEAR_NEF | UART_CLEAR_PEF |
                        UART_CLEAR_IDLEF);

  status = HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart,
                                        ctx->rx_buf[ctx->active_rx_buf],
                                        ctx->rx_buf_size);
  if (status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
  }

  return status;
}

static void App_HandleFrame(uint8_t port_id, const uint8_t *payload, uint8_t len)
{
  uint8_t next_head;

  if (len == 0U)
  {
    return;
  }

  next_head = (uint8_t)((resp_head + 1U) % RESPONSE_QUEUE_DEPTH);
  if (next_head == resp_tail)
  {
    response_queue_overflow_count++;   /* FIX: 记录响应队列溢出丢帧(原来是静默丢弃,无法诊断) */
    return;
  }

  /* FIX: 预留 1 字节给 '*',即使接近满帧也能追加标记,保证行为一致 */
  if (len > (uint8_t)(FRAME_MAX_PAYLOAD - 1U))
  {
    len = (uint8_t)(FRAME_MAX_PAYLOAD - 1U);
  }
  memcpy(response_queue[resp_head].payload, payload, len);
  response_queue[resp_head].payload[len++] = '*';
  response_queue[resp_head].port_id = port_id;
  response_queue[resp_head].len = len;
  resp_head = next_head;
}

static void App_TrySendNextResponse(void)
{
  ResponseItem *item;
  uint16_t frame_len;

  if (usart3_tx_dma_busy != 0U)
  {
    return;
  }

  if (resp_tail == resp_head)
  {
    return;
  }

  if ((usart3_tx_done_tick != 0U) &&
      ((HAL_GetTick() - usart3_tx_done_tick) < USART3_INTER_FRAME_GAP_MS))
  {
    return;
  }

  item = &response_queue[resp_tail];
  frame_len = App_BuildResponseFrame(usart3_tx_dma_buf, item->port_id, item->payload, item->len);
  if (frame_len == 0U)
  {
    resp_tail = (uint8_t)((resp_tail + 1U) % RESPONSE_QUEUE_DEPTH);
    return;
  }

  usart3_tx_dma_busy = 1U;
  usart3_tx_start_tick = HAL_GetTick();
  if (HAL_UART_Transmit_DMA(&huart3, usart3_tx_dma_buf, frame_len) != HAL_OK)
  {
    usart3_tx_dma_busy = 0U;
    return;
  }

  resp_tail = (uint8_t)((resp_tail + 1U) % RESPONSE_QUEUE_DEPTH);
}

static void App_CheckTxTimeout(void)
{
  if ((usart3_tx_dma_busy != 0U) &&
      ((HAL_GetTick() - usart3_tx_start_tick) > USART3_TX_TIMEOUT_MS))
  {
    (void)HAL_UART_AbortTransmit(&huart3);
    usart3_tx_dma_busy = 0U;
  }
}

static uint16_t App_BuildResponseFrame(uint8_t *buf, uint8_t port_id, const uint8_t *payload, uint8_t len)
{
  uint8_t checksum;
  uint16_t offset = 0U;
  uint8_t i;

  if (len > FRAME_MAX_PAYLOAD)
  {
    return 0U;
  }

  buf[offset++] = FRAME_SOF0;
  buf[offset++] = FRAME_SOF1;
  buf[offset++] = port_id;
  buf[offset++] = len;

  checksum = (uint8_t)(port_id + len);
  for (i = 0U; i < len; i++)
  {
    buf[offset++] = payload[i];
    checksum = (uint8_t)(checksum + payload[i]);
  }

  buf[offset++] = checksum;
  return offset;
}

static void App_QueueUsart3RxBytes(const uint8_t *payload, uint16_t len)
{
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    uint16_t next_head = (uint16_t)((usart3_rx_head + 1U) % USART3_RX_QUEUE_SIZE);

    if (next_head == usart3_rx_tail)
    {
      usart3_rx_overflow_count++;
      return;
    }

    usart3_rx_queue[usart3_rx_head] = payload[i];
    __DMB();                          /* FIX: 内存屏障,确保数据写入先于 head 更新对消费者可见 */
    usart3_rx_head = next_head;
  }
}

static uint8_t App_DequeueUsart3RxByte(uint8_t *data)
{
  uint32_t primask;

  if (usart3_rx_tail == usart3_rx_head)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (usart3_rx_tail == usart3_rx_head)
  {
    App_RestoreIrqState(primask);
    return 0U;
  }

  *data = usart3_rx_queue[usart3_rx_tail];
  usart3_rx_tail = (uint16_t)((usart3_rx_tail + 1U) % USART3_RX_QUEUE_SIZE);
  App_RestoreIrqState(primask);
  return 1U;
}

static void App_RestoreIrqState(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint8_t FrameParser_Input(FrameParser *parser, uint8_t data)
{
  switch (parser->state)
  {
    case FRAME_WAIT_SOF0:
      if (data == FRAME_SOF0)
      {
        parser->state = FRAME_WAIT_SOF1;
      }
      break;

    case FRAME_WAIT_SOF1:
      if (data == FRAME_SOF1)
      {
        parser->state = FRAME_WAIT_PORT;
      }
      else if (data != FRAME_SOF0)
      {
        parser->state = FRAME_WAIT_SOF0;
      }
      break;

    case FRAME_WAIT_PORT:
      parser->port_id = data;
      parser->checksum = data;
      parser->state = FRAME_WAIT_LEN;
      break;

    case FRAME_WAIT_LEN:
      if (data > FRAME_MAX_PAYLOAD)
      {
        parser->state = FRAME_WAIT_SOF0;
      }
      else
      {
        parser->len = data;
        parser->index = 0U;
        parser->checksum = (uint8_t)(parser->checksum + data);
        parser->state = (data == 0U) ? FRAME_WAIT_CHECKSUM : FRAME_WAIT_PAYLOAD;
      }
      break;

    case FRAME_WAIT_PAYLOAD:
      parser->payload[parser->index++] = data;
      parser->checksum = (uint8_t)(parser->checksum + data);
      if (parser->index >= parser->len)
      {
        parser->state = FRAME_WAIT_CHECKSUM;
      }
      break;

    case FRAME_WAIT_CHECKSUM:
      parser->state = FRAME_WAIT_SOF0;
      if (parser->checksum == data)
      {
        return 1U;
      }
      break;

    default:
      parser->state = FRAME_WAIT_SOF0;
      break;
  }

  return 0U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    usart3_tx_done_tick = HAL_GetTick();
    usart3_tx_dma_busy = 0U;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint8_t *completed_buf;

  if (huart != usart3_rx_ctx.huart)
  {
    return;
  }

  if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  if (Size > usart3_rx_ctx.rx_buf_size)
  {
    Size = usart3_rx_ctx.rx_buf_size;
  }

  completed_buf = usart3_rx_ctx.rx_buf[usart3_rx_ctx.active_rx_buf];
  usart3_rx_ctx.active_rx_buf ^= 1U;

  if (App_StartRxDma(&usart3_rx_ctx) != HAL_OK)
  {
    usart3_rx_restart_pending = 1U;
    uart_dma_restart_error_count++;
  }

  if (Size > 0U)
  {
    App_QueueUsart3RxBytes(completed_buf, Size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != usart3_rx_ctx.huart)
  {
    return;
  }

  __HAL_UART_CLEAR_FLAG(huart,
                        UART_CLEAR_OREF | UART_CLEAR_FEF |
                        UART_CLEAR_NEF | UART_CLEAR_PEF |
                        UART_CLEAR_IDLEF);

  if (App_StartRxDma(&usart3_rx_ctx) != HAL_OK)
  {
    usart3_rx_restart_pending = 1U;
    uart_dma_restart_error_count++;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
