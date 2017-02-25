/**
 * @file    SW_DP.c
 * @brief   SWD driver
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "main.h"
#include "stm32f3xx_hal.h"
#include "DAP.h"

// SWCLK/TCK I/O pin -------------------------------------

/** SWCLK/TCK I/O pin: Set Output to High.
Set the SWCLK/TCK DAP hardware I/O pin to high level.
*/
static void     PIN_SWCLK_TCK_SET(void)
{
  HAL_GPIO_WritePin(SWCLK_GPIO_Port, SWCLK_Pin, SET);
}

/** SWCLK/TCK I/O pin: Set Output to Low.
Set the SWCLK/TCK DAP hardware I/O pin to low level.
*/
static void     PIN_SWCLK_TCK_CLR(void)
{
  HAL_GPIO_WritePin(SWCLK_GPIO_Port, SWCLK_Pin, RESET);
}

// SWDIO/TMS Pin I/O --------------------------------------

/** SWDIO/TMS I/O pin: Set Output to High.
Set the SWDIO/TMS DAP hardware I/O pin to high level.
*/
static void     PIN_SWDIO_TMS_SET(void)
{
  HAL_GPIO_WritePin(SWDIO_GPIO_Port, SWDIO_Pin, SET);
}

/** SWDIO/TMS I/O pin: Set Output to Low.
Set the SWDIO/TMS DAP hardware I/O pin to low level.
*/
static  void     PIN_SWDIO_TMS_CLR(void)
{
  HAL_GPIO_WritePin(SWDIO_GPIO_Port, SWDIO_Pin, RESET);
}

/** SWDIO I/O pin: Get Input (used in SWD mode only).
\return Current status of the SWDIO DAP hardware I/O pin.
*/
static  uint32_t PIN_SWDIO_IN(void)
{
  return HAL_GPIO_ReadPin(SWDIO_GPIO_Port, SWDIO_Pin);
}

/** SWDIO I/O pin: Set Output (used in SWD mode only).
\param bit Output value for the SWDIO DAP hardware I/O pin.
*/
static  void     PIN_SWDIO_OUT(uint32_t bit)
{
  if (bit & 0x1) {
    HAL_GPIO_WritePin(SWDIO_GPIO_Port, SWDIO_Pin, SET);
  } else {
    HAL_GPIO_WritePin(SWDIO_GPIO_Port, SWDIO_Pin, RESET);
  }
}

/** SWDIO I/O pin: Switch to Output mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to output mode. This function is
called prior \ref PIN_SWDIO_OUT function calls.
*/
static  void     PIN_SWDIO_OUT_ENABLE(void)
{
#if 0
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = SWDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SWDIO_GPIO_Port, &GPIO_InitStruct);
#else
  GPIOB->MODER |= GPIO_MODER_MODER4_0;
#endif
}

/** SWDIO I/O pin: Switch to Input mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to input mode. This function is
called prior \ref PIN_SWDIO_IN function calls.
*/
static  void     PIN_SWDIO_OUT_DISABLE(void)
{
#if 0
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = SWDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWDIO_GPIO_Port, &GPIO_InitStruct);
#else
  GPIOB->MODER &= ~GPIO_MODER_MODER4_0;
#endif
}

#define PIN_SWCLK_SET PIN_SWCLK_TCK_SET
#define PIN_SWCLK_CLR PIN_SWCLK_TCK_CLR

#define SW_CLOCK_CYCLE()                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_WRITE_BIT(bit)               \
  PIN_SWDIO_OUT(bit);                   \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_READ_BIT(bit)                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  bit = PIN_SWDIO_IN();                 \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()
 
#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)


void SWJ_Sequence (uint32_t count, uint8_t *data) {
  uint32_t val;
  uint32_t n;

  val = 0;
  n = 0;
  while (count--) {
    if (n == 0) {
      val = *data++;
      n = 8;
    }
    if (val & 1) {
      PIN_SWDIO_TMS_SET();
    } else {
      PIN_SWDIO_TMS_CLR();
    }
    SW_CLOCK_CYCLE();
    val >>= 1;
    n--;
  }
}

// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t SWD_Transfer (uint32_t request, uint32_t *data) {                       \
  uint32_t ack;                                                                 \
  uint32_t bit;                                                                 \
  uint32_t val;                                                                 \
  uint32_t parity;                                                              \
                                                                                \
  uint32_t n;                                                                   \
                                                                                \
  /* Packet Request */                                                          \
  parity = 0;                                                                   \
  SW_WRITE_BIT(1);                      /* Start Bit */                         \
  bit = request >> 0;                                                           \
  SW_WRITE_BIT(bit);                    /* APnDP Bit */                         \
  parity += bit;                                                                \
  bit = request >> 1;                                                           \
  SW_WRITE_BIT(bit);                    /* RnW Bit */                           \
  parity += bit;                                                                \
  bit = request >> 2;                                                           \
  SW_WRITE_BIT(bit);                    /* A2 Bit */                            \
  parity += bit;                                                                \
  bit = request >> 3;                                                           \
  SW_WRITE_BIT(bit);                    /* A3 Bit */                            \
  parity += bit;                                                                \
  SW_WRITE_BIT(parity);                 /* Parity Bit */                        \
  SW_WRITE_BIT(0);                      /* Stop Bit */                          \
  SW_WRITE_BIT(1);                      /* Park Bit */                          \
                                                                                \
  /* Turnaround */                                                              \
  PIN_SWDIO_OUT_DISABLE();                                                      \
  for (n = DAP_Data.swd_conf.turnaround; n; n--) {                              \
    SW_CLOCK_CYCLE();                                                           \
  }                                                                             \
                                                                                \
  /* Acknowledge response */                                                    \
  SW_READ_BIT(bit);                                                             \
  ack  = bit << 0;                                                              \
  SW_READ_BIT(bit);                                                             \
  ack |= bit << 1;                                                              \
  SW_READ_BIT(bit);                                                             \
  ack |= bit << 2;                                                              \
                                                                                \
  if (ack == DAP_TRANSFER_OK) {         /* OK response */                       \
    /* Data transfer */                                                         \
    if (request & DAP_TRANSFER_RnW) {                                           \
      /* Read data */                                                           \
      val = 0;                                                                  \
      parity = 0;                                                               \
      for (n = 32; n; n--) {                                                    \
        SW_READ_BIT(bit);               /* Read RDATA[0:31] */                  \
        parity += bit;                                                          \
        val >>= 1;                                                              \
        val  |= bit << 31;                                                      \
      }                                                                         \
      SW_READ_BIT(bit);                 /* Read Parity */                       \
      if ((parity ^ bit) & 1) {                                                 \
        ack = DAP_TRANSFER_ERROR;                                               \
      }                                                                         \
      if (data) *data = val;                                                    \
      /* Turnaround */                                                          \
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
      PIN_SWDIO_OUT_ENABLE();                                                   \
    } else {                                                                    \
      /* Turnaround */                                                          \
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
      PIN_SWDIO_OUT_ENABLE();                                                   \
      /* Write data */                                                          \
      val = *data;                                                              \
      parity = 0;                                                               \
      for (n = 32; n; n--) {                                                    \
        SW_WRITE_BIT(val);              /* Write WDATA[0:31] */                 \
        parity += val;                                                          \
        val >>= 1;                                                              \
      }                                                                         \
      SW_WRITE_BIT(parity);             /* Write Parity Bit */                  \
    }                                                                           \
    /* Idle cycles */                                                           \
    n = DAP_Data.transfer.idle_cycles;                                          \
    if (n) {                                                                    \
      PIN_SWDIO_OUT(0);                                                         \
      for (; n; n--) {                                                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
    }                                                                           \
    PIN_SWDIO_OUT(1);                                                           \
    return (ack);                                                               \
  }                                                                             \
                                                                                \
  if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {              \
    /* WAIT or FAULT response */                                                \
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) != 0)) {  \
      for (n = 32+1; n; n--) {                                                  \
        SW_CLOCK_CYCLE();               /* Dummy Read RDATA[0:31] + Parity */   \
      }                                                                         \
    }                                                                           \
    /* Turnaround */                                                            \
    for (n = DAP_Data.swd_conf.turnaround; n; n--) {                            \
      SW_CLOCK_CYCLE();                                                         \
    }                                                                           \
    PIN_SWDIO_OUT_ENABLE();                                                     \
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) == 0)) {  \
      PIN_SWDIO_OUT(0);                                                         \
      for (n = 32+1; n; n--) {                                                  \
        SW_CLOCK_CYCLE();               /* Dummy Write WDATA[0:31] + Parity */  \
      }                                                                         \
    }                                                                           \
    PIN_SWDIO_OUT(1);                                                           \
    return (ack);                                                               \
  }                                                                             \
                                                                                \
  /* Protocol error */                                                          \
  for (n = DAP_Data.swd_conf.turnaround + 32 + 1; n; n--) {                     \
    SW_CLOCK_CYCLE();                   /* Back off data phase */               \
  }                                                                             \
  PIN_SWDIO_OUT_ENABLE();                                                       \
  PIN_SWDIO_OUT(1);                                                             \
  return (ack);                                                                 \
}
