// Kevin Kemper
//
// Basic timer running on TC1
////////////////////////////////////////////////////////////////////////////////
#ifndef STEP_TIMER_H
#define STEP_TIMER_H

#include <stdio.h>
#include <avr/io.h>

// stops the step timer counter and zeros it
void stepTimer_Stop(void);

// starts the step timer at clk/4 -> 8 MHz
void stepTimer_Start(void);

// inits the step timer, sets the timer ISR to a high-level interrupt
void initStepTimer(void);

// Get the current value of the step timer
uint16_t stepTimerValue(void);

// Clear the step timer and start counting again from zero
void resetStepTimer(void);

#endif
