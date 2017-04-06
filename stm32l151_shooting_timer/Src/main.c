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

typedef uint8_t bool;
#define false 0
#define true (!false)

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

const uint16_t digdata[] = {0xe700, 0x2100, 0xcb00, 0x6b00, 0x2d00, 0x6e00, 0xee00, 0x2300, 0xef00, 0x6f00};
const uint32_t resetdigit = 0xff00000f;

__IO uint8_t update;
__IO uint16_t leddat[] = {0x0000, 0x0000, 0x0000, 0x0000};

__IO uint16_t counts[10] = {0};
__IO uint32_t count_num;

const uint16_t buzz_sampl[] = {
  0xC20, 0xEF0, 0xC20, 0x380, 0x010, 0x3A0, 0xC30, 0xFF0, 0xBC0, 0x430,
  0x110, 0x5E0, 0xC00, 0xD80, 0x860, 0x3A0, 0x440, 0x950, 0xC00, 0x9F0,
  0x640, 0x4D0, 0x6A0, 0x960, 0xAD0, 0x8D0, 0x680, 0x5C0, 0x730, 0xAD0,
  0x9F0, 0x620, 0x400, 0x960, 0xB60, 0x8C0, 0x5F0, 0x8F0, 0xA80, 0x640,
  0x2B0, 0x4A0
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);

static void MX_COMP2_Init(void);

uint32_t xorshift(void)
{
  static uint32_t y = 2463534242;
  y = y ^ (y << 13); y = y ^ (y >> 17);
  return y = y ^ (y << 5);
}

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

static void stop_buzzer(void)
{
  HAL_TIM_Base_Stop(&htim2);
  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim9);

  HAL_DAC_DeInit(&hdac);
  HAL_TIM_Base_DeInit(&htim2);
  HAL_TIM_Base_DeInit(&htim9);
}

static void prepare_buzzer(void)
{
  stop_buzzer();

  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();

  __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_UPDATE);

  __HAL_TIM_SET_AUTORELOAD(&htim2, 8000000 / 16000);
  __HAL_TIM_SET_AUTORELOAD(&htim9, ((16000 / 2 + 41) / 42) * 42);
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

void start_buzzer(void)
{
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)(&buzz_sampl[0]), 42, DAC_ALIGN_12B_R);
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

  if (count_num) disp_num += 1;

  set_led(counts[disp_num]);

  do {
    update = 0;
    do {
      xorshift();
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    } while (!update);

    start_sw <<= 1;
    if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)) {
      start_sw += 1;
    }

    disp_sw <<= 1;
    if (HAL_GPIO_ReadPin(DISP_GPIO_Port, DISP_Pin)) {
      disp_sw += 1;
    }

    if (start_sw == 0xfe) break;

    if (disp_sw == 0xfe) {
      if (count_num) {
        if (disp_num > count_num-1) {
          disp_num = 0;
        }
        disp_num += 1;
        set_led(counts[disp_num]);
      }
    }

  } while (1);

  return false;
}

static bool run(void)
{
  uint8_t start_sw = 0;
  uint8_t disp_sw = 0;
  bool ret = false;

  prepare_buzzer();
  
  count_num = 0;
  update = 0;
  do {} while (!update);

  start_buzzer();
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  tim4_ic_enable();

  do {
    update = 0;
    do {
      xorshift();
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    } while (!update);

    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim4);

    if (count_num < 1) {
      set_led_blink(cnt, ((cnt % 100) < 50));
    } else {
      set_led_blink(counts[count_num], ((cnt % 100) < 50));
    }

    if (count_num == 9) break;
    
    start_sw <<= 1;
    if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)) {
      start_sw += 1;
    }

    disp_sw <<= 1;
    if (HAL_GPIO_ReadPin(DISP_GPIO_Port, DISP_Pin)) {
      disp_sw += 1;
    }

    if (start_sw == 0xfe) {
      ret = true;
      break;
    }

    if (disp_sw == 0xfe) break;

  } while (1);

  TIM4->CCER &= ~(TIM_CCER_CC4E);
  TIM4->DIER &= ~(TIM_DIER_CC4IE);

  HAL_COMP_Stop(&hcomp2);
  HAL_COMP_DeInit(&hcomp2);

  return ret;
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  HAL_TIM_Base_Start(&htim3);
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC3);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC4);

  HAL_TIM_Base_Start(&htim4);

  update = 0;
  leddat[0] = leddat[1] = leddat[2] = leddat[3] = 0;

  count_num = 0;

  while (1)
  {
    while ( idle() ) {}

    while ( run() ) {}
  }
}

/** System Clock Configuration */
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
    Error_Handler();
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

/* COMP2 init function */
static void MX_COMP2_Init(void)
{
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_VREFINT;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_PB4;
  hcomp2.Init.Output = COMP_OUTPUT_TIM4IC4;
  hcomp2.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* DAC init function */
static void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  sConfigOC.Pulse = 125-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 375-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 625-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 875-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
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
  htim9.Init.Period = 0;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim9, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMP_GPIO_Port, AMP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CA1_Pin|CA2_Pin|CA3_Pin|D3_Pin 
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|CA4_Pin|D1_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_Pin DISP_Pin DELAY_Pin SEL1_Pin 
                           SEL2_Pin */
  GPIO_InitStruct.Pin = START_Pin|DISP_Pin|DELAY_Pin|SEL1_Pin 
                          |SEL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AMP_Pin */
  GPIO_InitStruct.Pin = AMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CA1_Pin CA2_Pin CA3_Pin D3_Pin 
                           D4_Pin D5_Pin D6_Pin D7_Pin 
                           D8_Pin CA4_Pin D1_Pin D2_Pin */
  GPIO_InitStruct.Pin = CA1_Pin|CA2_Pin|CA3_Pin|D3_Pin 
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|CA4_Pin|D1_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
}
