
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "uart_serial.h"

#define RX_BUF_SZ       64  /* must be power of two */
#define TX_BUF_SZ       2048

static uint8_t rx_dma_buf[RX_BUF_SZ];
static uint8_t tx_dma_buf[TX_BUF_SZ];
static uint8_t tx_buf[TX_BUF_SZ];

TIM_HandleTypeDef htim4;
static void MX_TIM4_Init(void);

static UART_HandleTypeDef *huart;
static uint32_t rd_ptr;
static uint32_t wr_ptr;

#define DMA_WRITE_PTR ( (RX_BUF_SZ - __HAL_DMA_GET_COUNTER(huart->hdmarx)) & (RX_BUF_SZ - 1) )

void uart_serial_init(UART_HandleTypeDef *h)
{
  huart = h;
  HAL_UART_Receive_DMA(huart, rx_dma_buf, RX_BUF_SZ);

  /* Disable the UART Parity Error Interrupt */
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

  MX_TIM4_Init();
  HAL_TIM_Base_Start_IT(&htim4);

  rd_ptr = 0;
  wr_ptr = 0;
}

uint8_t uart_rx_is_empty(void) {
  if(rd_ptr == DMA_WRITE_PTR) {
    return 1;
  }
  return 0;
}

uint8_t uart_get(void) {
  uint8_t c = 0;
  if(rd_ptr != DMA_WRITE_PTR) {
    c = rx_dma_buf[rd_ptr++];
    rd_ptr &= (RX_BUF_SZ - 1);
  }
  return c;
}

void uart_put(uint8_t c)
{
  if (wr_ptr != TX_BUF_SZ) {
    tx_dma_buf[wr_ptr++] = c;
  }
}

void uart_write(const uint8_t *buf)
{
  uint8_t c;
  while ( (c = *buf++) != 0 ) uart_put(c);
}

void putch(uint8_t c)
{
  uart_put(c);
}

_ssize_t _write_r(struct _reent *r, int fd, const void *buf, size_t cnt)
{
  const uint8_t *p = buf;
  for (size_t pos=0; pos<cnt; pos++) {
    if (*p == '\n') {
      putch('\r');
    }
    putch(*p++);
  }

  return cnt;
}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==TIM4)
  {
    if (wr_ptr)
    {
      memcpy(tx_buf, tx_dma_buf, wr_ptr);
      HAL_UART_Transmit_DMA(huart, tx_buf, wr_ptr);
      wr_ptr = 0;
    }
  }
}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 72-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}