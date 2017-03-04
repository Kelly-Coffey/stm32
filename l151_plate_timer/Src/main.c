/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32l1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint16_t digdata[] = {0x00e7, 0x0021, 0x00cb, 0x006b, 0x002d, 0x06e, 0x00ee, 0x0023, 0x00ef, 0x006f};
const uint32_t resetdigit = 0x00ff0700;

__IO uint16_t leddat[] = {0x0000, 0x0000, 0x0000};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t xorshift(void)
{
  static uint32_t y = 2463534242;
  y = y ^ (y << 13); y = y ^ (y >> 17);
  return y = y ^ (y << 5);
}

void update_seg(void)
{
  xorshift();

  if (__HAL_TIM_GET_FLAG(&htim3, TIM_IT_CC1)) {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
    GPIOA->BSRR = resetdigit;
    GPIOA->BSRR = leddat[0] | 0x01000000;
  }
  else
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_IT_CC2)) {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
    GPIOA->BSRR = resetdigit;
    GPIOA->BSRR = leddat[1] | 0x02000000;
  }
  else
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_IT_CC3)) {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC3);
    GPIOA->BSRR = resetdigit;
    GPIOA->BSRR = leddat[2] | 0x04000000;
  }
}

uint32_t time_set(void)
{
  uint32_t cnt;
  do {
    update_seg();

    if (__HAL_TIM_GET_FLAG(&htim3, TIM_IT_UPDATE)) {
      __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
 
      cnt = __HAL_TIM_GET_COUNTER(&htim2);
      if (cnt >= 20) {
        leddat[1] = digdata[(cnt/2) % 10] | 0x0010 ;
        leddat[2] = digdata[(cnt/20) % 10];
      } else {
        leddat[1] = digdata[(cnt/2) % 10] | 0x0010 ;
        leddat[2] = 0x0000;
      }

      cnt = __HAL_TIM_GET_COUNTER(&htim4);
      leddat[0] = digdata[cnt/2];
    }

    if (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == RESET) {
      cnt = (__HAL_TIM_GET_COUNTER(&htim2)/2)*10 + __HAL_TIM_GET_COUNTER(&htim4)/2;
      if (cnt > 5) {
        break;
      }
    }
  } while (1);

  return cnt;
}

void countdown(uint32_t cnt)
{
  uint32_t t = 0;
  uint32_t buzz = 5;

  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  while ( !__HAL_TIM_GET_FLAG(&htim3, TIM_IT_UPDATE) ) {
    update_seg();
  }
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

  MX_TIM10_Init();
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  while (cnt) {

    update_seg();

    if (!__HAL_TIM_GET_FLAG(&htim3, TIM_IT_UPDATE) ) continue;
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

    if (++t < 10) continue;
    t = 0;

    if (buzz) {
      if (--buzz == 0) {
        HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
        HAL_TIM_Base_DeInit(&htim10);
      }
    }
    --cnt;
    leddat[0] = digdata[cnt % 10];
    leddat[1] = digdata[(cnt/10) % 10] | 0x0010;
    if (cnt >= 100) {
      leddat[2] = digdata[(cnt/100) % 10];
    } else {
      leddat[2] = 0x0000;
    }
  }
}

void plock(void)
{
  MX_TIM10_Init();
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, SET);

  uint32_t cnt = 50;

  while (cnt) {
    update_seg();

    if (!__HAL_TIM_GET_FLAG(&htim3, TIM_IT_UPDATE) ) continue;
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

    --cnt;
  }

  HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
  HAL_TIM_Base_DeInit(&htim10);

  cnt = 250;

  while (cnt) {
    update_seg();

    if (!__HAL_TIM_GET_FLAG(&htim3, TIM_IT_UPDATE) ) continue;
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

    --cnt;
  }

  HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, RESET);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  GPIO_InitTypeDef GPIO_InitStruct;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();

  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start(&htim3);
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  uint32_t count = 50;

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    __HAL_TIM_SET_COUNTER(&htim2, (count/10)*2);
    __HAL_TIM_SET_COUNTER(&htim4, (count%10)*2);

    count = time_set();

    if (HAL_GPIO_ReadPin(SEL_GPIO_Port, SEL_Pin)) {
      leddat[0] = leddat[1] = leddat[2] = 0x0008;

      uint32_t delay = (xorshift() % 200) + 150;
      __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
      while (delay) {
        update_seg();
        if ( !__HAL_TIM_GET_FLAG(&htim3, TIM_IT_UPDATE) ) continue;
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
        --delay;
      }
    }

    countdown(count);

    plock();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (100*2)-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 90-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 15;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 45;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 75;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = (10*2)-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 2250-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim10, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1125-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim10);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D0_Pin|D1_Pin|D2_Pin|D3_Pin 
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CA1_Pin|CA2_Pin|CA3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D3_Pin 
                           D4_Pin D5_Pin D6_Pin D7_Pin 
                           CA1_Pin CA2_Pin CA3_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin 
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |CA1_Pin|CA2_Pin|CA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB13 PB14 PB15 
                           PB4 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL_Pin SW_Pin */
  GPIO_InitStruct.Pin = SEL_Pin|SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LOCK_Pin */
  GPIO_InitStruct.Pin = LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOCK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
