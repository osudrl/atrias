#ifndef PWM_H
#define PWM_H

#include <avr/io.h>

#define PERIOD 20000 

void initPWM(PORT_t * port, TC1_t * timer, HIRES_t * hires, uint8_t pwm_pin, uint8_t dir_pin);
void disablePWM(TC1_t * timer);
void setPWM(uint16_t duty, uint8_t dir, PORT_t * port, TC1_t * timer, uint8_t dir_pin);

#endif