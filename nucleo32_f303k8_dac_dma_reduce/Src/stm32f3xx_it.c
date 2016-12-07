/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_dac1_ch1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;
// extern UART_HandleTypeDef huart2;

extern __IO HAL_UART_StateTypeDef uart2_gState;
extern __IO uint32_t uart2_ErrorCode;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel3 global interrupt.
*/
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac1_ch1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel7 global interrupt.
*/
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXT line 26.
*/
void USART2_IRQHandler(void)
{
  /* UART parity error interrupt occurred -------------------------------------*/
  if((USART2->ISR & USART_ISR_PE)  && (USART2->CR1 & USART_CR1_PEIE))
  {
    USART2->ICR = USART_ICR_PECF;
  }

  /* UART frame error interrupt occurred --------------------------------------*/
  if((USART2->ISR & USART_ISR_FE)  && (USART2->CR3 & USART_CR3_EIE))
  {
    USART2->ICR = USART_ICR_FECF;

    uart2_ErrorCode |= HAL_UART_ERROR_FE;
  }

  /* UART noise error interrupt occurred --------------------------------------*/
  if((USART2->ISR & USART_ISR_NE)  && (USART2->CR3 & USART_CR3_EIE))
  {
    USART2->ICR = USART_ICR_NCF;

    uart2_ErrorCode |= HAL_UART_ERROR_NE;
  }

  /* UART Over-Run interrupt occurred -----------------------------------------*/
  if((USART2->ISR & USART_ISR_ORE)  && (USART2->CR3 & USART_CR3_EIE))
  {
    USART2->ICR = USART_ICR_ORECF;

    uart2_ErrorCode |= HAL_UART_ERROR_ORE;
  }
#if 0
  /* UART wakeup from Stop mode interrupt occurred -------------------------------------*/
  if((USART2->ISR & USART_ISR_WUF)  && (USART2->CR3 & USART_CR3_WUFIE))
  {
    USART2->ICR = USART_ICR_WUCF;
    /* Set the UART state ready to be able to start again the process */
    huart2.gState = HAL_UART_STATE_READY;

    HAL_UARTEx_WakeupCallback(&huart2);
  }
#endif
  /* UART in mode Receiver ---------------------------------------------------*/
  if((USART2->ISR & USART_ISR_RXNE)  && (USART2->CR1 & USART_CR1_RXNEIE))
  {
//    UART_Receive_IT(&huart2);
  }
#if 0
  /* UART in mode Transmitter ------------------------------------------------*/
  if((USART2->ISR & USART_ISR_TXE)  && (USART2->CR1 & USART_CR1_TXEIE))
  {
    UART_Transmit_IT(&huart2);
  }
#endif
  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if((USART2->ISR & USART_ISR_TC)  && (USART2->CR1 & USART_CR1_TCIE))
  {
    /* Disable the UART Transmit Complete Interrupt */
    USART2->CR1 &= ~USART_CR1_TCIE;

    /* Tx process is ended, restore huart->gState to Ready */
    uart2_gState = HAL_UART_STATE_READY;

//    HAL_UART_TxCpltCallback(huart);
  }

  if(uart2_ErrorCode != HAL_UART_ERROR_NONE)
  {
    /* Set the UART state ready to be able to start again the Tx/Rx process */
    uart2_gState = HAL_UART_STATE_READY;

//    HAL_UART_ErrorCallback(&huart2);
  }  
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
