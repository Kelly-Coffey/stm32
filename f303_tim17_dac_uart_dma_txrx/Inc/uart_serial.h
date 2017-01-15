#ifndef __UART_SERIAL_H
#define __UART_SERIAL_H

#include "usart.h"

extern void msg_init(UART_HandleTypeDef *huart);
extern uint8_t msgrx_circ_buf_is_empty(void);
extern uint8_t msgrx_circ_buf_get(void);
extern void msgtx_transmit(void);
extern void msgtx_buf_put(uint8_t c);
extern void msgtx_buf_put_sz(const uint8_t *buf);

#endif
