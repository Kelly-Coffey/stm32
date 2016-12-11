/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim6.h"

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
