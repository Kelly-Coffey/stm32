/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
typedef uint8_t bool;
#define false 0
#define true (!false)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint16_t digdata[] = {0x07e0, 0x00c0, 0x0b60, 0x09e0, 0x0cc0,
                            0x0da0, 0x0fa0, 0x04f0, 0x0fe0, 0x0ce0};
const uint32_t resetdigit = 0x1fe0000f;

__IO uint8_t update;
__IO uint16_t leddat[] = {0x0000, 0x0000, 0x0000, 0x0000};

__IO uint16_t counts[10] = {0};
__IO uint32_t count_num;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_COMP2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void Init(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVC_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Configure the system clock */
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /**Initializes the CPU, AHB and APB busses clocks */
  /*------------------------------- HSE Configuration ------------------------*/
  SET_BIT(RCC->CR, RCC_CR_HSEON);

  /* Wait till HSE is ready */
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET) ;

  /* Increasing the number of wait states because of higher CPU frequency */
  if(FLASH_LATENCY_0 > (FLASH->ACR & FLASH_ACR_LATENCY))
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
  }

  /*-------------------------- HCLK Configuration --------------------------*/
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV1);

  /*------------------------- SYSCLK Configuration ---------------------------*/
  /* HSE is selected as System Clock Source */
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSE);

  while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSE);

  /* Decreasing the number of wait states because of lower CPU frequency */
  if(FLASH_LATENCY_0 < (FLASH->ACR & FLASH_ACR_LATENCY))
  {
      /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
  }

  /*-------------------------- PCLK1 Configuration ---------------------------*/
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV1);

  /*-------------------------- PCLK2 Configuration ---------------------------*/
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_HCLK_DIV1) << 3U));

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HSE_VALUE;

  /**Configure the Systick interrupt time */
  HAL_SYSTICK_Config(HSE_VALUE/8000);

  /**Configure the Systick */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
}

//void HAL_SYSTICK_Callback(void)
//{
//  update = 1;
//}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      GPIOB->BSRR = resetdigit;
      GPIOB->BSRR = leddat[0] | 0x00010000;
    } else
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      GPIOB->BSRR = resetdigit;
      GPIOB->BSRR = leddat[1] | 0x00020000;
    } else
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      GPIOB->BSRR = resetdigit;
      GPIOB->BSRR = leddat[2] | 0x00040000;
    } else
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
      GPIOB->BSRR = resetdigit;
      GPIOB->BSRR = leddat[3] | 0x00080000;
    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
      if (count_num < 9) {
        uint16_t cnt = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4);
        if ( (cnt - counts[count_num]) >= 10) {
          counts[++count_num] = cnt;

          cnt += 10;
          if (cnt >= 60000) cnt -= 50000;
        }
      }
    }
  }
}

void start_buzzer(void)
{
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim9);
}

void stop_buzzer(void)
{
  HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim9);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    update = 1;
  }

  else
  if (htim->Instance == TIM4) {
    __HAL_TIM_SET_COUNTER(&htim4, __HAL_TIM_GET_COUNTER(&htim4) + 10000);
  }

  else
  if (htim->Instance == TIM9) {
    stop_buzzer();
  }
}

void tim4_ic_enable(void)
{
  MX_COMP2_Init();
  HAL_COMP_Start(&hcomp2);

  /* Enable the Input Capture channel */
  TIM4->CCER |= TIM_CCER_CC4E;
  TIM4->DIER |= TIM_DIER_CC4IE;
}

static void set_led(uint16_t cnt)
{
  leddat[0] = digdata[cnt % 10];
  leddat[1] = digdata[(cnt / 10) % 10];
  leddat[2] = digdata[(cnt / 100) % 10] | 0x1000;
  if (cnt >= 1000) {
    leddat[3] = digdata[(cnt / 1000) % 10];
  } else {
    leddat[3] = 0;
  }  
}

static void set_led_blink(uint16_t cnt, bool dot)
{
  leddat[0] = digdata[cnt % 10];
  leddat[1] = digdata[(cnt / 10) % 10];
  if (dot) {
    leddat[2] = digdata[(cnt / 100) % 10] | 0x1000;
  } else {
    leddat[2] = digdata[(cnt / 100) % 10];    
  }
  if (cnt >= 1000) {
    leddat[3] = digdata[(cnt / 1000) % 10];
  } else {
    leddat[3] = 0;
  }  
}

static bool idle(void)
{
  uint32_t disp_num = 0;
  uint8_t start_sw = 0;
  uint8_t disp_sw = 0;

  if (count_num) disp_num = 1;

  set_led(counts[disp_num]);

  do {
    update = 0;
    do {
    } while (!update);
    
    start_sw <<= 1;
    if (HAL_GPIO_ReadPin(Start_GPIO_Port, Start_Pin)) {
      start_sw += 1;
    }
      
    if (start_sw == 0xfe) {
      break;
    }

  } while (1);

  return false;
}

static bool run(void)
{
  uint8_t start_sw = 0;
  uint8_t disp_sw = 0;
  
  start_buzzer();

  HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);
  count_num = 0;
  tim4_ic_enable();
  
  while (1)
  {
    update = 0;
    do {

    } while (!update);

    start_sw <<= 1;
    if (HAL_GPIO_ReadPin(Start_GPIO_Port, Start_Pin)) {
      start_sw += 1;
    }
    
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim4);
  
    if (count_num < 1) {
      set_led_blink(cnt, ((cnt % 100) < 50));
    } else {
      set_led_blink(counts[count_num], ((cnt % 100) < 50));
    }

    if (count_num == 9) break;
  }

  TIM4->CCER &= ~(TIM_CCER_CC4E);
  TIM4->DIER &= ~(TIM_DIER_CC4IE);

  HAL_COMP_Stop(&hcomp2);
  HAL_COMP_DeInit(&hcomp2);

  return false;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  Init();
  __HAL_RCC_COMP_CLK_ENABLE();
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
//  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM11_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_COMP2_Init();

  /* USER CODE BEGIN 2 */
  {
    TIM_IC_InitTypeDef sConfigIC;
  
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);
  }

  leddat[0] = leddat[1] = leddat[2] = leddat[3] = 0;
  
  HAL_TIM_Base_Start(&htim3);
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC3);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC4);

  HAL_TIM_Base_Start(&htim4);

  count_num = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    while ( idle() ) {}

    while ( run() ) {}
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* COMP2 init function */
static void MX_COMP2_Init(void)
{

  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_VREFINT;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_PB4;
  hcomp2.Init.Output = COMP_OUTPUT_TIM4IC4;
  hcomp2.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 25-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 75-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 125-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 175-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchronization(&htim9, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 2000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim11, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 800-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim11);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CA1_Pin|CA2_Pin|CA3_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|CA4_Pin|D0_Pin 
                          |D1_Pin|D2_Pin|D3_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CA1_Pin CA2_Pin CA3_Pin D5_Pin 
                           D6_Pin D7_Pin CA4_Pin D0_Pin 
                           D1_Pin D2_Pin D3_Pin D4_Pin */
  GPIO_InitStruct.Pin = CA1_Pin|CA2_Pin|CA3_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|CA4_Pin|D0_Pin 
                          |D1_Pin|D2_Pin|D3_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_Pin */
  GPIO_InitStruct.Pin = Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Start_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
