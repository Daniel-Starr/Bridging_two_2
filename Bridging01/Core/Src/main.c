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
#define BRIDGE_TX_PAYLOAD_SIZE         (FRAME_MAX_PAYLOAD - 1U)
#define UART_RX_DMA_BUFFER_SIZE        512U
#define BRIDGE_TX_QUEUE_DEPTH          128U
#define USART3_RX_QUEUE_SIZE           8192U
#define USART3_TX_FRAME_SIZE           (2U + 1U + 1U + FRAME_MAX_PAYLOAD + 1U)
#define FRAME_PARSE_TIMEOUT_MS         100U
#define USART3_INTER_FRAME_GAP_MS      1U
#define USART3_TX_TIMEOUT_MS           50U

#define PORT_ID_USART1                 1U
#define PORT_ID_UART4                  2U
#define PORT_ID_LPUART1                3U

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
  uint8_t port_id;
  uint8_t pc_port;
  volatile uint8_t restart_pending;   /* FIX: 每端口独立的 RX DMA 重启 pending 标志 */
} UartRxDmaContext;

typedef struct
{
  uint8_t port_id;
  uint8_t len;
  uint8_t payload[FRAME_MAX_PAYLOAD];
} BridgeTxItem;

/* FIX: PC 端口非阻塞 DMA 发送所需的每端口结构(替代原来的阻塞轮询) */
#define PC_TX_QUEUE_DEPTH              32U
#define PC_TX_TIMEOUT_MS               1000U

typedef struct
{
  uint8_t len;
  uint8_t data[FRAME_MAX_PAYLOAD];
} PcTxItem;

typedef struct
{
  UART_HandleTypeDef *huart;
  PcTxItem queue[PC_TX_QUEUE_DEPTH];
  uint8_t dma_buf[FRAME_MAX_PAYLOAD];
  volatile uint8_t head;
  volatile uint8_t tail;
  volatile uint8_t busy;
  volatile uint32_t tx_start_tick;
  volatile uint32_t overflow_count;
} PcTxPort;

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
static uint8_t usart1_rx_dma_buf[2][UART_RX_DMA_BUFFER_SIZE];
static uint8_t uart4_rx_dma_buf[2][UART_RX_DMA_BUFFER_SIZE];
static uint8_t lpuart1_rx_dma_buf[2][UART_RX_DMA_BUFFER_SIZE];
static uint8_t usart3_rx_dma_buf[2][UART_RX_DMA_BUFFER_SIZE];

static UartRxDmaContext usart1_rx_ctx = {
  &huart1, {usart1_rx_dma_buf[0], usart1_rx_dma_buf[1]},
  UART_RX_DMA_BUFFER_SIZE, 0U, PORT_ID_USART1, 1U
};
static UartRxDmaContext uart4_rx_ctx = {
  &huart4, {uart4_rx_dma_buf[0], uart4_rx_dma_buf[1]},
  UART_RX_DMA_BUFFER_SIZE, 0U, PORT_ID_UART4, 1U
};
static UartRxDmaContext lpuart1_rx_ctx = {
  &hlpuart1, {lpuart1_rx_dma_buf[0], lpuart1_rx_dma_buf[1]},
  UART_RX_DMA_BUFFER_SIZE, 0U, PORT_ID_LPUART1, 1U
};
static UartRxDmaContext usart3_rx_ctx = {
  &huart3, {usart3_rx_dma_buf[0], usart3_rx_dma_buf[1]},
  UART_RX_DMA_BUFFER_SIZE, 0U, 0U, 0U
};
static UartRxDmaContext *const rx_dma_contexts[] = {
  &usart1_rx_ctx,
  &uart4_rx_ctx,
  &lpuart1_rx_ctx,
  &usart3_rx_ctx
};

static FrameParser usart3_frame_parser = {0};
static BridgeTxItem bridge_tx_queue[BRIDGE_TX_QUEUE_DEPTH];
static uint8_t usart3_rx_queue[USART3_RX_QUEUE_SIZE];
static volatile uint8_t bridge_tx_head = 0U;
static volatile uint8_t bridge_tx_tail = 0U;
static volatile uint16_t usart3_rx_head = 0U;
static volatile uint16_t usart3_rx_tail = 0U;
static volatile uint32_t bridge_tx_overflow_count = 0U;
static volatile uint32_t usart3_rx_overflow_count = 0U;
static volatile uint32_t uart_dma_restart_error_count = 0U;
static uint8_t usart3_tx_dma_buf[USART3_TX_FRAME_SIZE];
static volatile uint8_t usart3_tx_dma_busy = 0U;
static volatile uint32_t usart3_tx_start_tick = 0U;
static volatile uint32_t usart3_tx_done_tick = 0U;
static uint32_t usart3_frame_parser_tick = 0U;

/* FIX: 三个 PC 端口的非阻塞 DMA 发送端口(index 0/1/2 对应 port_id 1/2/3) */
static PcTxPort pc_tx_ports[3] = {
  { .huart = &huart1 },
  { .huart = &huart4 },
  { .huart = &hlpuart1 }
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_InitBridge(void);
static void App_ProcessBridge(void);
static void App_StartAllRxDma(void);
static void App_RetryPendingRxDma(void);
static HAL_StatusTypeDef App_StartRxDma(UartRxDmaContext *ctx);
static UartRxDmaContext *App_GetRxDmaContext(UART_HandleTypeDef *huart);
static void App_TrySendNextBridgeFrame(void);
static void App_CheckTxTimeouts(void);
static void App_ProcessUsart3RxQueue(void);
static void App_HandleUsart3Frame(uint8_t port_id, const uint8_t *payload, uint8_t len);
static void App_SendUartString(USART_TypeDef *uart, const char *text);
static PcTxPort *App_GetPcTxPort(uint8_t port_id);
static void App_QueuePcTx(PcTxPort *port, const uint8_t *payload, uint8_t len);
static void App_TrySendNextPcFrame(PcTxPort *port);
static uint16_t App_BuildBridgeFrame(uint8_t *buf, uint8_t port_id, const uint8_t *payload, uint8_t len);
static uint8_t App_QueueBridgeTx(uint8_t port_id, const uint8_t *payload, uint16_t len);
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
  App_InitBridge();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_ProcessBridge();
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
  huart4.Init.BaudRate = 9600;
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
  huart1.Init.BaudRate = 115200;
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
static void App_InitBridge(void)
{
  App_SendUartString(USART1, "[Board1] USART1 Ready\r\n");
  App_SendUartString(UART4, "[Board1] UART4 Ready\r\n");
  App_SendUartString(LPUART1, "[Board1] LPUART1 Ready\r\n");
  App_StartAllRxDma();
}

static void App_ProcessBridge(void)
{
  App_RetryPendingRxDma();            /* FIX: 重试所有重启失败的 RX DMA(含三个 PC 端口,原来只重试 USART3) */

  App_ProcessUsart3RxQueue();
  App_CheckTxTimeouts();

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

  App_TrySendNextBridgeFrame();

  /* FIX: 非阻塞驱动三个 PC 端口的 DMA 发送(替代原来的阻塞 App_SendUartData) */
  App_TrySendNextPcFrame(&pc_tx_ports[0]);
  App_TrySendNextPcFrame(&pc_tx_ports[1]);
  App_TrySendNextPcFrame(&pc_tx_ports[2]);
}

static void App_StartAllRxDma(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(rx_dma_contexts) / sizeof(rx_dma_contexts[0])); i++)
  {
    if (App_StartRxDma(rx_dma_contexts[i]) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

static void App_RetryPendingRxDma(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(rx_dma_contexts) / sizeof(rx_dma_contexts[0])); i++)
  {
    if (rx_dma_contexts[i]->restart_pending != 0U)
    {
      if (App_StartRxDma(rx_dma_contexts[i]) == HAL_OK)
      {
        rx_dma_contexts[i]->restart_pending = 0U;
      }
    }
  }
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

static UartRxDmaContext *App_GetRxDmaContext(UART_HandleTypeDef *huart)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(rx_dma_contexts) / sizeof(rx_dma_contexts[0])); i++)
  {
    if (rx_dma_contexts[i]->huart == huart)
    {
      return rx_dma_contexts[i];
    }
  }

  return 0;
}

static void App_TrySendNextBridgeFrame(void)
{
  BridgeTxItem *item;
  uint16_t frame_len;

  if (usart3_tx_dma_busy != 0U)
  {
    return;
  }

  if (bridge_tx_tail == bridge_tx_head)
  {
    return;
  }

  if ((usart3_tx_done_tick != 0U) &&
      ((HAL_GetTick() - usart3_tx_done_tick) < USART3_INTER_FRAME_GAP_MS))
  {
    return;
  }

  item = &bridge_tx_queue[bridge_tx_tail];
  frame_len = App_BuildBridgeFrame(usart3_tx_dma_buf, item->port_id, item->payload, item->len);
  if (frame_len == 0U)
  {
    bridge_tx_tail = (uint8_t)((bridge_tx_tail + 1U) % BRIDGE_TX_QUEUE_DEPTH);
    return;
  }

  usart3_tx_dma_busy = 1U;
  usart3_tx_start_tick = HAL_GetTick();
  if (HAL_UART_Transmit_DMA(&huart3, usart3_tx_dma_buf, frame_len) != HAL_OK)
  {
    usart3_tx_dma_busy = 0U;
    return;
  }

  bridge_tx_tail = (uint8_t)((bridge_tx_tail + 1U) % BRIDGE_TX_QUEUE_DEPTH);
}

static void App_ProcessUsart3RxQueue(void)
{
  uint8_t data;

  while (App_DequeueUsart3RxByte(&data) != 0U)
  {
    if (FrameParser_Input(&usart3_frame_parser, data) != 0U)
    {
      App_HandleUsart3Frame(usart3_frame_parser.port_id,
                            usart3_frame_parser.payload,
                            usart3_frame_parser.len);
    }
  }
}

static void App_HandleUsart3Frame(uint8_t port_id, const uint8_t *payload, uint8_t len)
{
  PcTxPort *port = App_GetPcTxPort(port_id);

  if ((port != 0) && (len > 0U))
  {
    App_QueuePcTx(port, payload, len);   /* FIX: 入队非阻塞发送,不再逐字节阻塞主循环 */
  }
}

static PcTxPort *App_GetPcTxPort(uint8_t port_id)
{
  if ((port_id >= PORT_ID_USART1) && (port_id <= PORT_ID_LPUART1))
  {
    return &pc_tx_ports[port_id - 1U];   /* port_id 1/2/3 -> index 0/1/2 */
  }
  return 0;
}

static void App_QueuePcTx(PcTxPort *port, const uint8_t *payload, uint8_t len)
{
  uint8_t next_head = (uint8_t)((port->head + 1U) % PC_TX_QUEUE_DEPTH);

  if (next_head == port->tail)
  {
    port->overflow_count++;            /* 队列满,记录丢帧 */
    return;
  }

  if (len > FRAME_MAX_PAYLOAD)
  {
    len = FRAME_MAX_PAYLOAD;
  }

  memcpy(port->queue[port->head].data, payload, len);
  port->queue[port->head].len = len;
  __DMB();
  port->head = next_head;
}

static void App_TrySendNextPcFrame(PcTxPort *port)
{
  PcTxItem *item;

  if (port->busy != 0U)
  {
    return;
  }

  if (port->tail == port->head)
  {
    return;
  }

  item = &port->queue[port->tail];
  memcpy(port->dma_buf, item->data, item->len);

  port->busy = 1U;
  port->tx_start_tick = HAL_GetTick();
  if (HAL_UART_Transmit_DMA(port->huart, port->dma_buf, item->len) != HAL_OK)
  {
    port->busy = 0U;                   /* 启动失败,保留队列项,下次重试 */
    return;
  }

  port->tail = (uint8_t)((port->tail + 1U) % PC_TX_QUEUE_DEPTH);
}

static void App_CheckTxTimeouts(void)
{
  uint8_t i;

  if ((usart3_tx_dma_busy != 0U) &&
      ((HAL_GetTick() - usart3_tx_start_tick) > USART3_TX_TIMEOUT_MS))
  {
    (void)HAL_UART_AbortTransmit(&huart3);
    usart3_tx_dma_busy = 0U;
  }

  for (i = 0U; i < 3U; i++)
  {
    if ((pc_tx_ports[i].busy != 0U) &&
        ((HAL_GetTick() - pc_tx_ports[i].tx_start_tick) > PC_TX_TIMEOUT_MS))
    {
      (void)HAL_UART_AbortTransmit(pc_tx_ports[i].huart);
      pc_tx_ports[i].busy = 0U;
    }
  }
}

static void App_SendUartString(USART_TypeDef *uart, const char *text)
{
  while (*text != '\0')
  {
    while ((uart->ISR & USART_ISR_TXE_TXFNF) == 0U)
    {
    }

    uart->TDR = (uint8_t)(*text);
    text++;
  }
}

static uint16_t App_BuildBridgeFrame(uint8_t *buf, uint8_t port_id, const uint8_t *payload, uint8_t len)
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

static uint8_t App_QueueBridgeTx(uint8_t port_id, const uint8_t *payload, uint16_t len)
{
  uint16_t offset = 0U;

  while (offset < len)
  {
    uint16_t chunk = (uint16_t)(len - offset);
    uint8_t next_head = (uint8_t)((bridge_tx_head + 1U) % BRIDGE_TX_QUEUE_DEPTH);

    if (chunk > BRIDGE_TX_PAYLOAD_SIZE)
    {
      chunk = BRIDGE_TX_PAYLOAD_SIZE;
    }

    if (next_head == bridge_tx_tail)
    {
      bridge_tx_overflow_count++;
      return 0U;
    }

    bridge_tx_queue[bridge_tx_head].port_id = port_id;
    bridge_tx_queue[bridge_tx_head].len = (uint8_t)chunk;
    memcpy(bridge_tx_queue[bridge_tx_head].payload, &payload[offset], chunk);
    __DMB();                          /* FIX: 内存屏障,确保帧内容写入先于 head 更新对消费者可见 */
    bridge_tx_head = next_head;
    offset = (uint16_t)(offset + chunk);
  }

  return 1U;
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

/* App_GetPcUart 已移除:PC 路由现在通过 App_GetPcTxPort + DMA 非阻塞发送实现 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t i;

  if (huart->Instance == USART3)
  {
    usart3_tx_done_tick = HAL_GetTick();
    usart3_tx_dma_busy = 0U;
    return;
  }

  /* FIX: PC 端口 DMA 发送完成,清除对应 busy 标志 */
  for (i = 0U; i < 3U; i++)
  {
    if (pc_tx_ports[i].huart == huart)
    {
      pc_tx_ports[i].busy = 0U;
      break;
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  UartRxDmaContext *ctx = App_GetRxDmaContext(huart);
  uint8_t *completed_buf;

  if (ctx == 0)
  {
    return;
  }

  if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  if (Size > ctx->rx_buf_size)
  {
    Size = ctx->rx_buf_size;
  }

  completed_buf = ctx->rx_buf[ctx->active_rx_buf];
  ctx->active_rx_buf ^= 1U;

  if (App_StartRxDma(ctx) != HAL_OK)
  {
    ctx->restart_pending = 1U;        /* FIX: 所有端口都支持 pending 重试(原来只有 USART3) */
    uart_dma_restart_error_count++;
  }

  if (Size > 0U)
  {
    if (ctx->pc_port != 0U)
    {
      (void)App_QueueBridgeTx(ctx->port_id, completed_buf, Size);
    }
    else
    {
      App_QueueUsart3RxBytes(completed_buf, Size);
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  UartRxDmaContext *ctx = App_GetRxDmaContext(huart);

  if (ctx == 0)
  {
    return;
  }

  __HAL_UART_CLEAR_FLAG(huart,
                        UART_CLEAR_OREF | UART_CLEAR_FEF |
                        UART_CLEAR_NEF | UART_CLEAR_PEF |
                        UART_CLEAR_IDLEF);

  if (App_StartRxDma(ctx) != HAL_OK)
  {
    ctx->restart_pending = 1U;        /* FIX: 所有端口都支持 pending 重试(原来只有 USART3) */
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
