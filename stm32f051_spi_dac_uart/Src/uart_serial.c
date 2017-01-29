#include "uart_serial.h"

#define RX_BUF_SZ       64  /* must be power of two */
#define TX_BUF_SZ       64

static uint8_t rx_dma_buf[RX_BUF_SZ];
static uint8_t tx_dma_buf[TX_BUF_SZ];

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

void uart_transmit(void)
{
  while (huart->gState != HAL_UART_STATE_READY) ;
  HAL_UART_Transmit_DMA(huart, tx_dma_buf, wr_ptr);
  wr_ptr = 0;
}

void uart_put(uint8_t c)
{
  while (huart->gState != HAL_UART_STATE_READY) ;
  tx_dma_buf[wr_ptr++] = c;
  if (c == '\n' || wr_ptr == TX_BUF_SZ) {
    uart_transmit();
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
