/**
  ******************************************************************************
  * @file    USB_Device/CustomHID_Standalone/Src/usbd_customhid_if.c
  * @author  MCD Application Team
  * @version V1.7.0
  * @date    16-December-2016
  * @brief   USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
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
#include "usbd_customhid_if.h"
#include "DAP.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t CustomHID_Init     (void);
static int8_t CustomHID_DeInit   (void);
static int8_t CustomHID_OutEvent (uint8_t* event_idx);
/* Private variables ---------------------------------------------------------*/

extern USBD_HandleTypeDef hUsbDevice;

uint8_t SendBuffer[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];

static uint8_t DAP_ReportDesc[USBD_DAP_REPORT_DESC_SIZE] =
{
  0x06, 0x00, 0xFF,      /* USAGE_PAGE (Vendor Page: 0xFF00) */
  0x09, 0x01,            /* USAGE (Demo Kit)               */
  0xa1, 0x01,            /* COLLECTION (Application)       */
  /* 7 */

  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */
  0x26, 0xFF, 0x00,      /*     LOGICAL_MAXIMUM (255)      */           
  0x75, 0x08,            /*     REPORT_SIZE (8 bit)        */    
  0x95, 0x40,            /*     REPORT_COUNT (0x40)        */
  /* 16 */

  0x09, 0x01,            /* USAGE (Demo Kit)               */
  0x81, 0x02,            /* INPUT (Array)                  */
  0x95, 0x40,            /*     REPORT_COUNT (0x40)        */
  /* 22 */

  0x09, 0x01,            /* USAGE (Demo Kit)               */
  0x91, 0x02,            /* OUTPUT (Array)                 */
  0x95, 0x01,            /*     REPORT_COUNT (1)           */
  /* 28 */

  0x09, 0x01,            /* USAGE (Demo Kit)               */
  0xB1, 0x02,            /*     FEATURE (Array)            */
  /* 32 */

  0xc0                   /*     END_COLLECTION             */
};

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops = 
{
  DAP_ReportDesc,
  CustomHID_Init,
  CustomHID_DeInit,
  CustomHID_OutEvent,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CustomHID_Init
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CustomHID_Init(void)
{
  DAP_Setup();
  return (0);
}

/**
  * @brief  CustomHID_DeInit
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CustomHID_DeInit(void)
{
  return (0);
}

/**
  * @brief  CustomHID_OutEvent
  *         Manage the CUSTOM HID class Out Event    
  * @param  request: received HID report
  */
static int8_t CustomHID_OutEvent  (uint8_t* request)
{
  TIM4->DIER &= ~TIM_DIER_UIE;

  uint32_t length = 0;

  memset(SendBuffer, 0, 64);

  length = DAP_ProcessCommand(request, SendBuffer);
  
  USBD_CUSTOM_HID_SendReport(&hUsbDevice, SendBuffer, 64);

  TIM4->DIER |= TIM_DIER_UIE;
  return (0);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
