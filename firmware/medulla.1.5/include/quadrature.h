// Kevin Kemper
#ifndef QUADRATURE_H
#define QUADRATURE_H

#include <stdio.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define PORT_ENCODER		PORTB
#define ENC_A				2
#define ENC_B				3
#define ENC_A_bm			(1<<ENC_A)
#define ENC_B_bm			(1<<ENC_B)
#define TC_ENC				TCF0
#define ENC_EVSYS_CHMUX_A	EVSYS_CHMUX_PORTB_PIN2_gc
#define TIMER_MID			32767
#define TIMER_TOP			0xFFFF

int32_t inc_encoder_base;

#define GetCaptureValue(_tc)  ( _tc.CCA )

void initQuad();

#endif