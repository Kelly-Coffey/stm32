#ifndef __UART_SERIAL_H
#define __UART_SERIAL_H

#include "usart.h"

extern void uart_serial_init(UART_HandleTypeDef *huart);
extern uint8_t uart_rx_is_empty(void);
extern uint8_t uart_get(void);
extern void uart_transmit(void);
extern void uart_put(uint8_t c);
extern void uart_write(const uint8_t *buf);

#endif
