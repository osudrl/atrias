/*
Xmega USART Drivers Header
Uses atmel application note drivers

Daniel Sidlauskas Miller

Make sure to link usart_driver.c when compiling

Functions more explained in source file
*/

#include "avr_compiler.h"
#include "usart_driver.h"


void uartInit(USART_data_t * title,USART_t * interface, unsigned int baudrate);

void sendstring(USART_data_t * uart, char *buffer);

void sendchar(USART_data_t * uart, char buffer);
