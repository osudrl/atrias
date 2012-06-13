// Kevin Kemper
// Modifications by Kit Morton
//
////////////////////////////////////////////////////////////////////////////////
#ifndef LIMITSW_H
#define LIMITSW_H

#include <stdio.h>
#include <avr/io.h>

PORT_t * _limitSWPort;

// disable the limit switch interrupt
void limitDis();

// Enable the limit switch interupt
void limitEn();

// initilize the limit switch inputs
void initLimitSW(PORT_t* port);

uint8_t checkLimitSW(void);

#endif // !LIMITSW_H
