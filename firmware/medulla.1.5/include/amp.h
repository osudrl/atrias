#ifndef AMP_H
#define AMP_H

#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"

#define PERIOD 20000 

void initAmp(PORT_t * port, USART_t * amp_usart, TC1_t * timer, HIRES_t * hires, uint8_t pwm_pin, uint8_t dir_pin);
void enableAmp(USART_t * amp_usart);
void enablePWM(TC1_t * timer);
void disablePWM(TC1_t * timer);
void setPWM(uint16_t duty, uint8_t dir, PORT_t * port, TC1_t * timer, uint8_t dir_pin);

#endif