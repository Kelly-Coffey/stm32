/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
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
#include "usbd_cdc_if.h"

#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
uint32_t TxReadPtr;

extern USBD_HandleTypeDef hUsbDevice;
extern UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim3;

#define DMA_WRITE_PTR ( (APP_TX_DATA_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx)) & (APP_TX_DATA_SIZE - 1) )


  
static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

static void MX_TIM3_Init(void);
static void ComPort_Config(void);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = 
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,  
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  HAL_UART_Receive_DMA(&huart1, UserTxBufferFS, APP_RX_DATA_SIZE);

  MX_TIM3_Init();
  HAL_TIM_Base_Start_IT(&htim3);

  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDevice, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDevice, UserRxBufferFS);

  TxReadPtr = 0;

  return (USBD_OK);
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  HAL_TIM_Base_Stop_IT(&htim3);
  HAL_TIM_Base_DeInit(&htim3);

  /* Stop UART DMA Rx request if ongoing */
  if ((huart1.RxState == HAL_UART_STATE_BUSY_RX) &&
      (HAL_IS_BIT_SET(huart1.Instance->CR3, USART_CR3_DMAR)))
  {
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    /* Abort the UART DMA Rx channel */
    if(huart1.hdmarx != NULL)
    {
      HAL_DMA_Abort(huart1.hdmarx);
    }

    CLEAR_BIT(huart1.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);

    /* At end of Rx process, restore huart1.RxState to Ready */
    huart1.RxState = HAL_UART_STATE_READY;
  }

  return (USBD_OK);
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
 
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
 
    break;

  case CDC_SET_COMM_FEATURE:
 
    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */ 
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:   
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];
    
    /* Set the new configuration */
    ComPort_Config();
    break;

  case CDC_GET_LINE_CODING:     
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;
    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:
 
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
}

extern void HAL_TIM_PeriodElapsedCallback_2(TIM_HandleTypeDef *htim);

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==TIM3)
  {
    uint32_t buffsize;
    uint32_t writeptr = DMA_WRITE_PTR;

    if (TxReadPtr != writeptr)
    {
      if (TxReadPtr > writeptr)
      {
        buffsize = APP_TX_DATA_SIZE - TxReadPtr;
      }
      else
      {
        buffsize = writeptr - TxReadPtr;
      }

      USBD_CDC_SetTxBuffer(&hUsbDevice, (uint8_t*)&UserTxBufferFS[TxReadPtr], buffsize);

    
      if(USBD_CDC_TransmitPacket(&hUsbDevice) == USBD_OK)
      {
        TxReadPtr += buffsize;
        if (TxReadPtr == APP_RX_DATA_SIZE)
        {
          TxReadPtr = 0;
        }
      }
    }
  }
  else
  {
    HAL_TIM_PeriodElapsedCallback_2(htim);
  }
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
  char buff[64];
  memcpy(buff, Buf, *Len);
  buff[*Len] = 0;
  printf("CDC Receive: %s, %d\n", buff, *Len);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);

//  while (huart1.gState != HAL_UART_STATE_READY) ;
  HAL_UART_Transmit_DMA(&huart1, Buf, *Len);
  return (USBD_OK);
}

extern void HAL_UART_TxCpltCallback_2(UART_HandleTypeDef *huart);

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);

    printf("CDC Transmit Cplt\n");
    /* Initiate next USB packet transfer once UART completes transfer (transmitting data over Tx line) */
    USBD_CDC_ReceivePacket(&hUsbDevice);
  }
  else
  {
    HAL_UART_TxCpltCallback_2(huart);
  }
}


/* TIM3 init function */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 72-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  ComPort_Config
  *         Configure the COM Port with the parameters received from host.
  * @param  None.
  * @retval None.
  * @note   When a configuration is not supported, a default value is used.
  */
static void ComPort_Config(void)
{
  if(HAL_UART_DeInit(&huart1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* set the Stop bit */
  switch (LineCoding.format)
  {
  case 0:
    huart1.Init.StopBits = UART_STOPBITS_1;
    break;
  case 2:
    huart1.Init.StopBits = UART_STOPBITS_2;
    break;
  default :
    huart1.Init.StopBits = UART_STOPBITS_1;
    break;
  }
  
  /* set the parity bit*/
  switch (LineCoding.paritytype)
  {
  case 0:
    huart1.Init.Parity = UART_PARITY_NONE;
    break;
  case 1:
    huart1.Init.Parity = UART_PARITY_ODD;
    break;
  case 2:
    huart1.Init.Parity = UART_PARITY_EVEN;
    break;
  default :
    huart1.Init.Parity = UART_PARITY_NONE;
    break;
  }
  
  /*set the data type : only 8bits and 9bits is supported */
  switch (LineCoding.datatype)
  {
  case 0x07:
    /* With this configuration a parity (Even or Odd) must be set */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    break;
  case 0x08:
    if(huart1.Init.Parity == UART_PARITY_NONE)
    {
      huart1.Init.WordLength = UART_WORDLENGTH_8B;
    }
    else 
    {
      huart1.Init.WordLength = UART_WORDLENGTH_9B;
    }
    
    break;
  default :
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    break;
  }
  
  huart1.Init.BaudRate = LineCoding.bitrate;
  huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart1.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&huart1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  HAL_UART_Receive_DMA(&huart1, UserTxBufferFS, APP_RX_DATA_SIZE);
  TxReadPtr = 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

