#include "stm32f3xx_hal.h"

/* Bits position in  in the CFGR register */
#define RCC_CFGR_HPRE_BITNUMBER           POSITION_VAL(RCC_CFGR_HPRE)

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

/*  RCC_OscInitTypeDef RCC_OscInitStruct; */
/*  RCC_ClkInitTypeDef RCC_ClkInitStruct; */

    /**Initializes the CPU, AHB and APB busses clocks 
    */
/*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
*/
  /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
  __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(16);

  __HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSI, RCC_PLL_MUL8);

  /* Enable the main PLL. */
  __HAL_RCC_PLL_ENABLE();

  while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == RESET)
  {
  }
    
    /**Initializes the CPU, AHB and APB busses clocks 
    */
/*
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  {
    Error_Handler();
  }
*/
  /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /*-------------------------- HCLK Configuration --------------------------*/
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV1);
  
  /*------------------------- SYSCLK Configuration ---------------------------*/ 
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);

  while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
  {
  }

  /*-------------------------- PCLK1 Configuration ---------------------------*/ 
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV4);
  
  /*-------------------------- PCLK2 Configuration ---------------------------*/ 
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_HCLK_DIV1) << 3));
  
  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> RCC_CFGR_HPRE_BITNUMBER];

  /* Configure the source of time base considering new system clocks settings*/
  HAL_InitTick (TICK_INT_PRIORITY);
  
      
    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

