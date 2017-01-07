
#include "usart2.h"
#include "gpio.h"

/*!< UART or USART CR1 fields of parameters set by UART_SetConfig API */
#define UART_CR1_FIELDS  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | \
                                     USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8))

#define UART_BUFSIZE 128 /* 2^n (64,128,256,512...) */

__IO uint8_t TX_Buff[UART_BUFSIZE];
__IO unsigned int TX_head;
__IO unsigned int TX_tail;
__IO uint8_t RX_Buff[UART_BUFSIZE];
__IO unsigned int RX_head;
__IO unsigned int RX_tail;

void USART2_UART_Init(void)
{
  UART2_Init();

  /* Disable the Peripheral */
  USART2->CR1 &= ~USART_CR1_UE;

  /*-------------------------- USART CR1 Configuration -----------------------*/
  /* Clear M, PCE, PS, TE, RE and OVER8 bits and configure
   *  the UART Word Length, Parity, Mode and oversampling  */
  MODIFY_REG(USART2->CR1, UART_CR1_FIELDS, UART_WORDLENGTH_8B | UART_PARITY_NONE | UART_MODE_TX_RX | UART_OVERSAMPLING_16);

  /*-------------------------- USART CR2 Configuration -----------------------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits according
   * to huart->Init.StopBits value */
  MODIFY_REG(USART2->CR2, USART_CR2_STOP, UART_STOPBITS_1);

  /*-------------------------- USART CR3 Configuration -----------------------*/
  /* Configure
   * - UART HardWare Flow Control: set CTSE and RTSE bits according
   *   to huart->Init.HwFlowCtl value
   * - one-bit sampling method versus three samples' majority rule according
   *   to huart->Init.OneBitSampling */
  MODIFY_REG(USART2->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT), UART_HWCONTROL_NONE | UART_ONE_BIT_SAMPLE_DISABLE);

  USART2->BRR = (uint16_t)(UART_DIV_SAMPLING16(HAL_RCC_GetPCLK1Freq(), 38400));

  /* In asynchronous mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  USART2->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  USART2->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

  /* Enable the Peripheral */
  USART2->CR1 |=  USART_CR1_UE;

  /* TEACK and REACK */
  while ( !(USART2->ISR & USART_ISR_TEACK) );
  while ( !(USART2->ISR & USART_ISR_REACK) );

  TX_head = TX_tail = 0;
  RX_head = RX_tail = 0;

  /* Enable Receive Data register not empty interrupt */
  USART2->CR1 |= USART_CR1_RXNEIE;
}

void UART2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  __HAL_RCC_USART2_CLK_ENABLE();

  /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA15     ------> USART2_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral interrupt init */
  HAL_NVIC_SetPriority(USART2_IRQn, 4, 0);
  NVIC_EnableIRQ(USART2_IRQn);
}

void UART2_DeInit(void)
{
  /* Peripheral clock disable */
  __HAL_RCC_USART2_CLK_DISABLE();

  /* USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA15     ------> USART2_RX
  */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_15);

  /* Peripheral interrupt Deinit*/
  NVIC_DisableIRQ(USART2_IRQn);
}

void UART2_Transmit(uint8_t c)
{
  unsigned int tmp_t = (TX_tail + 1) & (UART_BUFSIZE - 1);

  while (TX_head == tmp_t) ;

  NVIC_DisableIRQ(USART2_IRQn);
  TX_Buff[TX_tail] = c;
  TX_tail = tmp_t;
  NVIC_EnableIRQ(USART2_IRQn);

  /* Enable Transmit Data Register empty interrupt */
  USART2->CR1 |= USART_CR1_TXEIE;
}

void UART2_Transmit_String(const char *buff)
{
  while (*buff)
  {
    UART2_Transmit(*buff++);
  }
}

uint8_t UART2_IsDataReceived(void)
{
  return (RX_head != RX_tail);
}

uint8_t UART2_Receive(void)
{
  uint8_t c;
  unsigned int tmp_h = RX_head;

  while (tmp_h == RX_tail) ;

  NVIC_DisableIRQ(USART2_IRQn);
  c = RX_Buff[tmp_h++];
  RX_head = tmp_h & (UART_BUFSIZE - 1);
  NVIC_EnableIRQ(USART2_IRQn);

  return c;
}

/* handles USART2 global interrupt / USART2 wake-up interrupt through EXT line 26. */
void USART2_IRQHandler(void)
{
  uint32_t isr = USART2->ISR;

  if (isr & USART_ISR_RXNE)
  {
    unsigned int tmp_t = (RX_tail + 1) & (UART_BUFSIZE-1);
    unsigned int tmp_h = RX_head;

    if (tmp_t != tmp_h) {
      RX_Buff[RX_tail] = USART2->RDR;
      RX_tail = tmp_t;
    } else {
      /* overflow */
    }
  }

  if (isr & USART_ISR_TXE)
  {
    unsigned int tmp_t = TX_tail;
    unsigned int tmp_h = TX_head;

    if (tmp_t == tmp_h) {
      /* Transfer completed. */
      /* Disable Transmit Data Register empty interrupt */
      USART2->CR1 &= ~USART_CR1_TXEIE;
    } else {
      USART2->TDR = TX_Buff[tmp_h];
      TX_head = (tmp_h + 1) & (UART_BUFSIZE - 1);
    }
  }
}


