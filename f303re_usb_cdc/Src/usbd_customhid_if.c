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
#include "uart_serial.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t CustomHID_Init     (void);
static int8_t CustomHID_DeInit   (void);
static int8_t CustomHID_OutEvent (uint8_t* event_idx);
/* Private variables ---------------------------------------------------------*/

uint8_t SendBuffer[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
extern USBD_HandleTypeDef hUsbDevice;

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

uint32_t Info(uint8_t* request)
{
  uint32_t length = 2;
  uint32_t id = request[1];

  SendBuffer[0] = 0x00;
  switch (id) {
    case 1: /* VENDOR */
      memcpy(SendBuffer+2, "STMicroelectronics", 19);
      SendBuffer[1] = 19;
      length += 19;
      break;

    case 2: /* PRODUCT */
      memcpy(SendBuffer+2, "STM32 CMSIS-DAP", 16);
      SendBuffer[1] = 16;
      length += 16;
      break;

    case 3: /* SERIAL NUM */
      memcpy(SendBuffer+2, "00000001", 9);
      SendBuffer[1] = 9;
      length += 9;
      break;

    case 4: /* FW Version */
      memcpy(SendBuffer+2, "1.0", 4);
      SendBuffer[1] = 4;
      length += 4;
      break;

    case 5: /* DEVICE VENDOR */
      memcpy(SendBuffer+2, "STMicroelectronics", 19);
      SendBuffer[1] = 19;
      length += 19;
      break;

    case 6: /* DEVICE NAME */
      memcpy(SendBuffer+2, "Nucleo F303RE", 14);
      SendBuffer[1] = 14;
      length += 14;
      break;

    case 0xf0: /* CAPABILITIES */
      SendBuffer[1] = 1;
      SendBuffer[2] = 0x01; /* SWD */
      length += 1;
      break;

    case 0xfe: /* PACKET COUNT */
      SendBuffer[1] = 1;
      SendBuffer[2] = 1;
      length += 1;
      break;

    case 0xff: /* PACKET SIZE */
      SendBuffer[1] = 2;
      SendBuffer[2] = 64;
      SendBuffer[3] = 0;
      length += 2;
      break;

    default:
      SendBuffer[1] = 0xff;
      break;
  }

  return length;
}

uint32_t LED(uint8_t* request)
{
  SendBuffer[0] = 0x01;
  SendBuffer[1] = 0;
  return 2;
}

uint32_t Connect(uint8_t* request)
{
  SendBuffer[0] = 0x02;
  SendBuffer[1] = DAP_PORT_DISABLED;
  if (request[1] == DAP_PORT_AUTODETECT || request[1] == DAP_PORT_SWD) {
    SendBuffer[1] = DAP_PORT_SWD;
  }
  return 2;
}

uint32_t Disonnect(uint8_t* request)
{
  SendBuffer[0] = 0x03;
  SendBuffer[1] = 0;
  return 2;
}

uint32_t TransferConfigure(uint8_t* request)
{
  uint8_t cycle = request[1];
  uint16_t retry = *(uint16_t*)(&request[2]);

  SendBuffer[0] = 0x04;
  SendBuffer[1] = 0;
  return 2;
}

uint32_t SWJ_Pins(uint8_t* request)
{
  uint32_t value = request[1];
  uint32_t select = request[2];
  uint32_t waittime_us = *(uint32_t*)(&request[3]);

  SendBuffer[0] = 0x10;
  SendBuffer[1] = 0x03;
  return 2;
}

uint32_t SWJ_Clock(uint8_t* request)
{
  SendBuffer[0] = 0x11;
  SendBuffer[1] = 0;
  return 2;
}

uint32_t SWJ_Sequence(uint8_t* request)
{
  SendBuffer[0] = 0x12;
  SendBuffer[1] = 0;
  return 2;
}

uint32_t SWD_Configure(uint8_t* request)
{
  SendBuffer[0] = 0x13;
  SendBuffer[1] = 0;
  return 2;
}

/**
  * @brief  CustomHID_OutEvent
  *         Manage the CUSTOM HID class Out Event    
  * @param  request: received HID report
  */
static int8_t CustomHID_OutEvent  (uint8_t* request)
{
  uint32_t req = request[0];
  uint32_t length = 0;

  switch (req) {
    case 0x00:
      length = Info(request);
      break;

    case 0x01:
      length = LED(request);
      break;

    case 0x02:
      length = Connect(request);
      break;

    case 0x03:
      length = Disonnect(request);
      break;

    case 0x04:
      length = TransferConfigure(request);
      break;

    case 0x10:
      length = SWJ_Pins(request);
      break;

    case 0x11:
      length = SWJ_Clock(request);
      break;

    case 0x12:
      length = SWJ_Sequence(request);
      break;

    case 0x13:
      length = SWD_Configure(request);
      break;

    default:
      length = 1;
      SendBuffer[0] = 0xff;
      break;
  }

  USBD_CUSTOM_HID_SendReport(&hUsbDevice, SendBuffer, length);
  return (0);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
