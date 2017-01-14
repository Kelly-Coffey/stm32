#include "uart_serial.h"

#define CIRC_BUF_SZ       64  /* must be power of two */
#define BUF_SZ            64

static uint8_t rx_dma_circ_buf[CIRC_BUF_SZ];
static uint8_t tx_dma_buf[BUF_SZ];

static UART_HandleTypeDef *huart_cobs;
static uint32_t rd_ptr;
static uint32_t wr_ptr;

#define DMA_WRITE_PTR ( (CIRC_BUF_SZ - __HAL_DMA_GET_COUNTER(huart_cobs->hdmarx)) & (CIRC_BUF_SZ - 1) )

void msg_init(UART_HandleTypeDef *huart)
{
  huart_cobs = huart;
  HAL_UART_Receive_DMA(huart_cobs, rx_dma_circ_buf, CIRC_BUF_SZ);

  /* These uart interrupts halt any ongoing transfer if an error occurs, disable them */
  /* Disable the UART Parity Error Interrupt */
  __HAL_UART_DISABLE_IT(huart_cobs, UART_IT_PE);
  /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_DISABLE_IT(huart_cobs, UART_IT_ERR);

  rd_ptr = 0;
  wr_ptr = 0;
}

uint8_t msgrx_circ_buf_is_empty(void) {
  if(rd_ptr == DMA_WRITE_PTR) {
    return 1;
  }
  return 0;
}

uint8_t msgrx_circ_buf_get(void) {
  uint8_t c = 0;
  if(rd_ptr != DMA_WRITE_PTR) {
    c = rx_dma_circ_buf[rd_ptr++];
    rd_ptr &= (CIRC_BUF_SZ - 1);
  }
  return c;
}

void msgtx_transmit(void)
{
  while (huart_cobs->gState != HAL_UART_STATE_READY) ;
  HAL_UART_Transmit_DMA(huart_cobs, tx_dma_buf, wr_ptr);
  wr_ptr = 0;
}

void msgtx_buf_put(uint8_t c)
{
  tx_dma_buf[wr_ptr++] = c;
  if (c == '\n' || wr_ptr == BUF_SZ) {
    msgtx_transmit();
  }
}

void putch(uint8_t c)
{
  msgtx_buf_put(c);
}

void msgtx_buf_put_sz(const uint8_t *buf)
{
  uint8_t c;
  while ( (c = *buf++) != 0 ) msgtx_buf_put(c);
}
