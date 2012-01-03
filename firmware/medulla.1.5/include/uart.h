#ifndef UART_H
#define UART_H

#include <stdio.h>
#include <avr/io.h>

void initUART(PORT_t *port, USART_t *usart, short baud);

void UARTWriteChar(USART_t *usart, uint8_t data);

uint8_t UARTReadChar(USART_t *usart);

#endif

