#include "stm32f3xx_hal.h"
#include "main.h"

#include <string.h>

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

void UART2_SetConfig(void);

/* USART2 init function */
void USART2_UART_DMA_Init(DMA_Channel_TypeDef *dmach)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  RCC->APB1ENR |=  RCC_APB1ENR_USART2EN;

  /* Init the low level hardware : GPIO, CLOCK */
  
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

  /* Peripheral UART2 init*/

  /* Disable the Peripheral */
  USART2->CR1 &= ~USART_CR1_UE;

  UART2_SetConfig();

  /* In asynchronous mode, the following bits must be kept cleared:
      - LINEN and CLKEN bits in the USART_CR2 register,
      - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  USART2->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  USART2->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

  /* Enable the Peripheral */
  USART2->CR1 |=  USART_CR1_UE;

  /* Wait until TEACK flag is set */
  while( !(USART2->ISR & USART_ISR_TEACK) ) ;

  /* Wait until REACK flag is set */
  while( !(USART2->ISR & USART_ISR_REACK) ) ;

  /* Enable the UART transmit DMA channel */

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

  /* Peripheral DMA init*/
/*  MODIFY_REG(dmach->CCR,
            DMA_CCR_PL    | DMA_CCR_MSIZE  | DMA_CCR_PSIZE  |
            DMA_CCR_MINC  | DMA_CCR_PINC   | DMA_CCR_CIRC   |
            DMA_CCR_DIR,
            DMA_PRIORITY_LOW | DMA_MDATAALIGN_BYTE | DMA_PDATAALIGN_BYTE |
            DMA_MINC_ENABLE | DMA_PINC_DISABLE | DMA_NORMAL | 
            DMA_MEMORY_TO_PERIPH);
*/
  /* Get the CR register value */
  uint32_t tmp = DMA1_Channel7->CCR;
  
  /* Clear PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR bits */
  tmp &= ((uint32_t)~(DMA_CCR_PL    | DMA_CCR_MSIZE  | DMA_CCR_PSIZE  | \
                      DMA_CCR_MINC  | DMA_CCR_PINC   | DMA_CCR_CIRC   | \
                      DMA_CCR_DIR));
  
  /* Prepare the DMA Channel configuration */
  tmp |=  DMA_MEMORY_TO_PERIPH |
          DMA_PINC_DISABLE     | DMA_MINC_ENABLE     |
          DMA_PDATAALIGN_BYTE  | DMA_MDATAALIGN_BYTE |
          DMA_NORMAL           | DMA_PRIORITY_LOW;

  /* Write to DMA Channel CR register */
  DMA1_Channel7->CCR = tmp;
}


/* Send data in DMA mode. */
void UART2_Transmit_DMA(DMA_Channel_TypeDef *dmach, uint8_t *pData, uint16_t Size)
{
  /* Disable the peripheral */
  dmach->CCR &= ~DMA_CCR_EN;

  /* Configure the source, destination address and the data length */
  dmach->CNDTR = Size;
  dmach->CMAR = (uint32_t)pData;
  dmach->CPAR = (uint32_t)&(USART2->TDR);

  /* Enable the transfer complete,  Half transfer complete, transfer Error interrupt */
  dmach->CCR |= DMA_IT_TC | DMA_IT_HT | DMA_IT_TE;

  /* Enable the Peripheral */
  dmach->CCR |= DMA_CCR_EN;

  /* Clear the TC flag in the ICR register */
  USART2->ICR = UART_CLEAR_TCF;

  /* Enable the DMA transfer for transmit request by setting the DMAT bit
     in the UART CR3 register */
  USART2->CR3 |= USART_CR3_DMAT;

  return;
}

/**
  * @brief Configure the UART peripheral.
  * @param huart: UART handle.
  * @retval HAL status
  */
void UART2_SetConfig(void)
{
  /*-------------------------- USART CR1 Configuration -----------------------*/
  /* Clear M, PCE, PS, TE, RE and OVER8 bits and configure
   *  the UART Word Length, Parity, Mode and oversampling */
  MODIFY_REG(USART2->CR1, 
             USART_CR1_M | USART_CR1_PCE | USART_CR1_PS |
             USART_CR1_TE | USART_CR1_RE |
             USART_CR1_OVER8, 
             UART_WORDLENGTH_8B | UART_PARITY_NONE |
             UART_MODE_TX_RX |  UART_OVERSAMPLING_16);

  /*-------------------------- USART CR2 Configuration -----------------------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits */
  MODIFY_REG(USART2->CR2, USART_CR2_STOP, UART_STOPBITS_1);

  /*-------------------------- USART CR3 Configuration -----------------------*/
  /* Configure
   * - UART HardWare Flow Control: set CTSE and RTSE bits
   * - one-bit sampling method versus three samples' majority rule */
  MODIFY_REG(USART2->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT), 
             UART_HWCONTROL_NONE | UART_ONE_BIT_SAMPLE_DISABLE);

  /*-------------------------- USART BRR Configuration -----------------------*/
  USART2->BRR = (uint16_t)(UART_DIV_SAMPLING16(HAL_RCC_GetPCLK1Freq(), 38400));
}

/**
* @brief This function handles DMA1 channel7 global interrupt.
*/

/**
  * @brief  Handles DMA interrupt request.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.  
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* Transfer Error Interrupt management ***************************************/
  if(DMA1->ISR & DMA_FLAG_TE7)
  {
    if(DMA1_Channel7->CCR & DMA_IT_TE)
    {
      /* Disable the transfer error interrupt */
      DMA1_Channel7->CCR &= ~DMA_IT_TE;
    
      /* Clear the transfer error flag */
      DMA1->IFCR = DMA_FLAG_TE7;

      /* Do Callback */
    }
  }

  /* Half Transfer Complete Interrupt management ******************************/
  if(DMA1->ISR & DMA_FLAG_HT7)
  {
    if(DMA1_Channel7->CCR & DMA_IT_HT)
    { 
      /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
      if( !(DMA1_Channel7->CCR & DMA_CCR_CIRC) )
      {
        /* Disable the half transfer interrupt */
        DMA1_Channel7->CCR &= ~DMA_IT_HT;
      }
      /* Clear the half transfer complete flag */
      DMA1->IFCR = DMA_FLAG_HT7;
      
      /* Do Callback */
    }
  }
  
  /* Transfer Complete Interrupt management ***********************************/
  if(DMA1->ISR & DMA_FLAG_TC7)
  {
    if(DMA1_Channel7->CCR & DMA_IT_TC)
    {
      if( !(DMA1_Channel7->CCR & DMA_CCR_CIRC) )
      {
        /* Disable the transfer complete interrupt */
        DMA1_Channel7->CCR &= ~DMA_IT_TC;
      }
      /* Clear the transfer complete flag */
      DMA1->IFCR = DMA_FLAG_TC7;

      if( !(DMA1_Channel7->CCR & DMA_CCR_CIRC) )
      {
        /* Disable the DMA transfer for transmit request by resetting the DMAT bit
    　     in the UART CR3 register */
        USART2->CR3 &= (uint32_t)~((uint32_t)USART_CR3_DMAT);

        /* Enable the UART Transmit Complete Interrupt */
        USART2->CR1 |= (1U << 5);
      } else {
        /* Circular mode callback */
      }
    }
  }
}  
