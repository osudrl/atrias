// Kevin Kemper
//
// Basic timer running on TC1
////////////////////////////////////////////////////////////////////////////////
#ifndef STEP_TIMER_H
#define STEP_TIMER_H

#include <stdio.h>
#include <avr/io.h>

// stops the step timer counter and zeros it
void timer_Stop(TC1_t * timer);

// starts the step timer
void timer_Start(TC1_t * timer, uint8_t prescaler);

// Get the current value of the step timer
uint16_t timerValue(TC1_t * timer);

// Clear the step timer and start counting again from zero
void resetTimer(TC1_t * timer);

#endif
