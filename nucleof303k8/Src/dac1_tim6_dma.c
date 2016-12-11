/**
  ******************************************************************************
  * File Name          : DAC.c
  * Description        : This file provides code for the configuration
  *                      of the DAC instances.
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
#include "dac1_tim6_dma.h"

#include "gpio.h"

void DAC1_MspInit(void);

HAL_DMA_StateTypeDef DAC1_State;

/* DAC1 init function */
void DAC1_Init(void)
{
  uint32_t tmpreg1 = 0, tmpreg2 = 0;

  /* Init the low level hardware */
  DAC1_MspInit();
  
  /* DAC channel OUT1 config */

  /* Get the DAC CR value */
  tmpreg1 = DAC1->CR;

  /* Output Buffer (BOFF1) control */
  tmpreg1 &= ~(((uint32_t)(DAC_CR_MAMP1 | DAC_CR_WAVE1 | DAC_CR_TSEL1 | DAC_CR_TEN1 | DAC_CR_BOFF1)) << DAC_CHANNEL_1);
  tmpreg2 = (DAC_TRIGGER_T6_TRGO | DAC_OUTPUTBUFFER_ENABLE);    

  /* Calculate CR register value depending on DAC_Channel */
  tmpreg1 |= tmpreg2 << DAC_CHANNEL_1;
  /* Write to DAC CR */
  DAC1->CR = tmpreg1;
  
  /* Disable wave generation */
  DAC1->CR &= ~(DAC_CR_WAVE1 << DAC_CHANNEL_1);
  
  DAC1_State = HAL_DMA_STATE_READY;    
}

void DAC1_MspInit(void)
{
  DMA_HandleTypeDef hdma_dac1_ch1;
  memset(&hdma_dac1_ch1, 0, sizeof(DMA_HandleTypeDef));

  GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  __HAL_RCC_DAC1_CLK_ENABLE();
  
  /**DAC1 GPIO Configuration    
    PA4     ------> DAC1_OUT1 
  */
  GPIO_InitStruct.Pin = DAC1_OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC1_OUT1_GPIO_Port, &GPIO_InitStruct);

  /* Peripheral DMA init*/
  hdma_dac1_ch1.Instance = DMA1_Channel3;
  hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_dac1_ch1.Init.Mode = DMA_NORMAL;
  hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
  {
    while(1) ;
  }

  __HAL_DMA_REMAP_CHANNEL_ENABLE(HAL_REMAPDMA_TIM6_DAC1_CH1_DMA1_CH3);

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

void DAC1_DeInit(void)
{
  DMA_HandleTypeDef hdma_dac1_ch1;
  memset(&hdma_dac1_ch1, 0, sizeof(DMA_HandleTypeDef));

  hdma_dac1_ch1.Instance = DMA1_Channel3;

  /* Peripheral clock disable */
  __HAL_RCC_DAC1_CLK_DISABLE();
  
  /**DAC1 GPIO Configuration    
    PA4     ------> DAC1_OUT1 
  */
  HAL_GPIO_DeInit(DAC1_OUT1_GPIO_Port, DAC1_OUT1_Pin);

  /* Peripheral DMA DeInit*/
  HAL_DMA_DeInit(&hdma_dac1_ch1);
} 


/* TIM6 init function */
void TIM6_Init(uint32_t freq)
{
  uint32_t tmpcr2;  
  uint32_t tmpsmcr;
  
  /* Peripheral clock enable */
  __HAL_RCC_TIM6_CLK_ENABLE();
  
  /* Set TIM Time Base Unit parameters ---------------------------------------*/

  /* Set the Autoreload value */
  TIM6->ARR = (uint32_t)(SystemCoreClock / freq) ;
 
  /* Set the Prescaler value */
  TIM6->PSC = (uint32_t)0;

  /* Generate an update event to reload the Prescaler */
  TIM6->EGR = TIM_EGR_UG;

 /* Get the TIM6 CR2 register value */
  tmpcr2 = TIM6->CR2;

  /* Get the TIM6 SMCR register value */
  tmpsmcr = TIM6->SMCR;

  /* Reset the MMS Bits */
  tmpcr2 &= ~TIM_CR2_MMS;
  /* Select the TRGO source */
  tmpcr2 |=  TIM_TRGO_UPDATE;

  /* Reset the MSM Bit */
  tmpsmcr &= ~TIM_SMCR_MSM;
  /* Set master mode */
  tmpsmcr |= TIM_MASTERSLAVEMODE_DISABLE;
  
  /* Update TIM6 CR2 */
  TIM6->CR2 = tmpcr2;
  
  /* Update TIM6 SMCR */
  TIM6->SMCR = tmpsmcr;
}

void TIM6_DeInit()
{
  /* Peripheral clock disable */
  __HAL_RCC_TIM6_CLK_DISABLE();
} 


void DAC1_Start_DMA(uint32_t* pData, uint32_t Length, uint32_t Alignment)
{
  uint32_t tmpreg = 0;
    
  /* Enable the selected DAC channel1 DMA request */
  DAC1->CR |= (DAC_CR_DMAEN1 << DAC_CHANNEL_1);

  /* Case of use of channel 1 */
  switch(Alignment)
  {
    case DAC_ALIGN_12B_R:
      /* Get DHR12R1 address */
      tmpreg = (uint32_t)&DAC1->DHR12R1;
      break;
    case DAC_ALIGN_12B_L:
      /* Get DHR12L1 address */
      tmpreg = (uint32_t)&DAC1->DHR12L1;
      break;
    case DAC_ALIGN_8B_R:
      /* Get DHR8R1 address */
      tmpreg = (uint32_t)&DAC1->DHR8R1;
      break;
    default:
      break;
  }
 
  /* Change DMA peripheral state */  
  DAC1_State = HAL_DMA_STATE_BUSY;  

  /* Disable the peripheral */
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;
  
  /* Configure DMA Channel data length */
  DMA1_Channel3->CNDTR = Length;
  
  /* Configure DMA Channel destination address */
  DMA1_Channel3->CPAR = (uint32_t)tmpreg;
  
  /* Configure DMA Channel source address */
  DMA1_Channel3->CMAR = (uint32_t) pData;
    
  /* Enable the transfer complete interrupt */
  DMA1_Channel3->CCR |= DMA_IT_TC;

  /* Enable the Peripheral */
  DMA1_Channel3->CCR |= DMA_CCR_EN;
 
  /* Enable the Peripheral */
  DAC1->CR |= (DAC_CR_EN1 << DAC_CHANNEL_1);
}


void DAC1_Stop_DMA(void)
{
  /* Disable the selected DAC channel DMA request */
  DAC1->CR &= ~(DAC_CR_DMAEN1 << DAC_CHANNEL_1);

  /* Disable the Peripheral */
  DAC1->CR &= ~(DAC_CR_EN1 << DAC_CHANNEL_1);
  
  /* Disable the DMA channel */
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;

  /* Change the DMA state */
  DAC1_State = HAL_DMA_STATE_READY;    
}

/**
* @brief This function handles DMA1 channel3 global interrupt.
*/
void DMA1_Channel3_IRQHandler(void)
{
  if (DMA1->ISR & DMA_FLAG_TC3) {
    /* Disable the transfer complete interrupt */
    DMA1_Channel3->CCR &= ~DMA_IT_TC;

    /* Clear the transfer complete flag */
    DMA1->IFCR = DMA_FLAG_TC3;
    
    /* Change the DMA state */
    DAC1_State = HAL_DMA_STATE_READY;    
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
