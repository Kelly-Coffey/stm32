/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

typedef uint8_t bool;
#define false 0
#define true (!false)

#define MESSAGE_BUTTON_START 0
#define MESSAGE_BUTTON_DISP  1
#define MESSAGE_EXIT_RUN    99

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId mainTaskHandle;
osThreadId buttonTaskHandle;
osThreadId counterTaskHandle;
osThreadId captureTaskHandle;
osMessageQId buttonQueueHandle;
osMessageQId counterQueueHandle;
osMessageQId captureQueueHandle;
osMutexId counterMutexHandle;
osSemaphoreId captureSemHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_COMP2_Init(void);
void StartMainTask(void const * argument);
void StartButtonTask(void const * argument);
void StartCounterTask(void const * argument);
void StartCaptureTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
const uint16_t digdata[] = {0xe700, 0x2100, 0xcb00, 0x6b00, 0x2d00, 0x6e00, 0xee00, 0x2300, 0xef00, 0x6f00};
const uint32_t resetdigit = 0xff00000f;

__IO uint16_t leddat[] = {0x0000, 0x0000, 0x0000, 0x0000};

__IO uint16_t counts[10] = {0};
__IO uint32_t count_num;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim4);
    if (cnt >= 60000) {
      cnt -= 50000;
      __HAL_TIM_SET_COUNTER(&htim4, cnt);
    }
    osMessagePut(counterQueueHandle, cnt, 0);
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
      TIM4->CCER &= ~(TIM_CCER_CC4E);
      TIM4->DIER &= ~(TIM_DIER_CC4IE);

      uint16_t cnt = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4);
      osMessagePut(captureQueueHandle, cnt, 0);
    }
  }
}

void set_led(uint16_t cnt)
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

void set_led_blink(uint16_t cnt, bool dot)
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
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3);
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC3);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC4);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of counterMutex */
  osMutexDef(counterMutex);
  counterMutexHandle = osMutexCreate(osMutex(counterMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of captureSem */
  osSemaphoreDef(captureSem);
  captureSemHandle = osSemaphoreCreate(osSemaphore(captureSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, StartMainTask, osPriorityNormal, 0, 128);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of buttonTask */
  osThreadDef(buttonTask, StartButtonTask, osPriorityLow, 0, 128);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask), NULL);

  /* definition and creation of counterTask */
  osThreadDef(counterTask, StartCounterTask, osPriorityAboveNormal, 0, 128);
  counterTaskHandle = osThreadCreate(osThread(counterTask), NULL);

  /* definition and creation of captureTask */
  osThreadDef(captureTask, StartCaptureTask, osPriorityAboveNormal, 0, 128);
  captureTaskHandle = osThreadCreate(osThread(captureTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of buttonQueue */
  osMessageQDef(buttonQueue, 1, uint16_t);
  buttonQueueHandle = osMessageCreate(osMessageQ(buttonQueue), NULL);

  /* definition and creation of counterQueue */
  osMessageQDef(counterQueue, 1, uint16_t);
  counterQueueHandle = osMessageCreate(osMessageQ(counterQueue), NULL);

  /* definition and creation of captureQueue */
  osMessageQDef(captureQueue, 1, uint16_t);
  captureQueueHandle = osMessageCreate(osMessageQ(captureQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* COMP2 init function */
static void MX_COMP2_Init(void)
{

  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_VREFINT;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_PB4;
  hcomp2.Init.Output = COMP_OUTPUT_TIM4IC4;
  hcomp2.Init.Mode = COMP_MODE_LOWSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
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
  htim3.Init.Prescaler = 800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  sConfigOC.Pulse = 12-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 37-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 62-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 87-1;
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
  htim4.Init.Period = 65536 - 1;
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
  HAL_GPIO_WritePin(GPIOB, CA1_Pin|CA2_Pin|CA3_Pin|D3_Pin 
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|CA4_Pin|D1_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_Pin DISP_Pin DELAY_Pin */
  GPIO_InitStruct.Pin = START_Pin|DISP_Pin|DELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartMainTask function */
void StartMainTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  uint32_t disp_num = 0;
  count_num = 0;
  set_led(0);

  /* Infinite loop */
  for(;;)
  {
    osEvent evt = osMessageGet(buttonQueueHandle, 180000);

    if (evt.status == osEventMessage) {
      if (evt.value.v == MESSAGE_BUTTON_START) {
        count_num = 0;
        set_led(0);

        MX_TIM4_Init();

        __HAL_TIM_DISABLE(&htim3);

        /* Generate an update event to reload the Prescaler */
        TIM3->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
        __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

        HAL_TIM_Base_Start(&htim4);
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);

        __HAL_TIM_SET_COUNTER(&htim4, 0);

        /* Comparator rising event -> TIM4 IC4 -> ISR */
        MX_COMP2_Init();
        HAL_COMP_Start(&hcomp2);

        osSemaphoreWait(captureSemHandle, osWaitForever);

        /* Enable the Input Capture channel */
        TIM4->CCER |= TIM_CCER_CC4E;
        TIM4->DIER |= TIM_DIER_CC4IE;

        /* Start Counter Now! */
        __HAL_TIM_ENABLE(&htim3);


        /* running... wait button or timeout */
        evt = osMessageGet(buttonQueueHandle, osWaitForever);


        osSemaphoreRelease(captureSemHandle);

        TIM4->CCER &= ~(TIM_CCER_CC4E);
        TIM4->DIER &= ~(TIM_DIER_CC4IE);

        HAL_COMP_Stop(&hcomp2);
        HAL_COMP_DeInit(&hcomp2);

        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
        HAL_TIM_Base_Stop(&htim4);
        HAL_TIM_Base_DeInit(&htim4);

        if (count_num) {
          disp_num = 1;
          set_led(counts[1]);
        } else {
          disp_num = 0;
          set_led(0);
        }
      }
      else
      if (count_num && evt.value.v == MESSAGE_BUTTON_DISP) {
        if (++disp_num > count_num) disp_num = 1;
        set_led(counts[disp_num]);
      }
    }
  }
  /* USER CODE END 5 */ 
}

/* StartButtonTask function */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN StartButtonTask */
  uint8_t sw_start = 0x00;
  uint8_t sw_disp = 0x00;

  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    sw_start <<= 1;
    if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)) sw_start |= 1;
    sw_disp <<= 1;
    if (HAL_GPIO_ReadPin(DISP_GPIO_Port, DISP_Pin)) sw_disp |= 1;

    if (sw_start == 0xfe) {
      osMessagePut(buttonQueueHandle, MESSAGE_BUTTON_START, 10);
    }
    if (sw_disp == 0xfe) {
      osMessagePut(buttonQueueHandle, MESSAGE_BUTTON_DISP, 10);
    }
  }
  /* USER CODE END StartButtonTask */
}

/* StartCounterTask function */
void StartCounterTask(void const * argument)
{
  /* USER CODE BEGIN StartCounterTask */
  /* Infinite loop */
  for(;;)
  {
    osEvent evt = osMessageGet(counterQueueHandle, osWaitForever);

    if (evt.status == osEventMessage) {
      uint16_t cnt = evt.value.v;

      osMutexWait(counterMutexHandle, osWaitForever);
      if (count_num) {
        set_led_blink(counts[count_num], ((cnt % 100) < 50));
      } else {
        set_led_blink(cnt, ((cnt % 100) < 50));
      }
      osMutexRelease(counterMutexHandle);
    }

  }
  /* USER CODE END StartCounterTask */
}

/* StartCaptureTask function */
void StartCaptureTask(void const * argument)
{
  /* USER CODE BEGIN StartCaptureTask */
  /* Infinite loop */
  for(;;)
  {
    osEvent evt = osMessageGet(captureQueueHandle, osWaitForever);

    if (evt.status == osEventMessage) {
      uint16_t cnt = evt.value.v;

      osMutexWait(counterMutexHandle, osWaitForever);
      volatile uint32_t cn = count_num;
      if (cn < 9) {
        counts[++cn] = cnt;
        count_num = cn;
      }
      osMutexRelease(counterMutexHandle);

      if (cn == 9) {
        osMessagePut(buttonQueueHandle, MESSAGE_EXIT_RUN, osWaitForever);
      } else {
        if (osSemaphoreWait(captureSemHandle, 100) != osOK) {
          TIM4->CCER |= TIM_CCER_CC4E;
          TIM4->DIER |= TIM_DIER_CC4IE;
        } else {
          osSemaphoreRelease(captureSemHandle);
        }
      }
    }
  }
  /* USER CODE END StartCaptureTask */
}

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
