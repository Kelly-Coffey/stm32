
#include "stdio.h"
#include "main.h"
#include "uart_serial.h"

#define TX_BUF_SZ       256

static uint8_t tx_dma_buf[TX_BUF_SZ];

static UART_HandleTypeDef *huart;
static uint32_t wr_ptr;

void uart_serial_init(UART_HandleTypeDef *h)
{
  huart = h;

  /* Disable the UART Parity Error Interrupt */
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

  wr_ptr = 0;
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
