// Kevin Kemper
#ifndef MENIAL_IO_H
#define MENIAL_IO_H

// All limit switches are on port K
#define PORT_LIMIT		PORTK
#define LIMIT_ENABLE_bm	((1<<7) |(1<<6) |(1<<5) |(1<<4) |(1<<3) | (1<<2)| (1<<1)| (1<<0))
#define MOTOR_LIMIT_bm	((1<<1) | (1<<0))
#define SPRING_LIM0_bm	(1<<2)
#define SPRING_LIM1_bm	(1<<3)
#define LIMIT_VECT		PORTK_INT0_vect

// LEDs
#define PORT_LED 		PORTJ
#define TC_LED			TCC0
#define TC_LED_OVF_vect	TCC0_OVF_vect
#define TC_LED_R_vect	TCC0_CCA_vect
#define TC_LED_G_vect	TCC0_CCB_vect
#define TC_LED_B_vect	TCC0_CCC_vect
#define LED_R			2
#define LED_G			4
#define LED_B			3
#define LED_R_bm		(1<<LED_R)
#define LED_G_bm		(1<<LED_G)
#define LED_B_bm		(1<<LED_B)


// panic
#define PORT_PANIC		PORTJ
#define PANIC_VECT		PORTJ_INT0_vect
#define PANIC			0
#define PANIC_SENSE		1
#define PANIC_bm		(1<<PANIC)
#define PANIC_SENSE_bm	(1<<PANIC_SENSE)

// ADC
#define PORT_VOLTAGE	PORTA
#define ADC_VOLTAGE		ADCA
#define ADC_LOGIC		0
#define ADC_POW			1
#define ADC_LOGIC_bm	(1<<ADC_LOGIC)
#define ADC_POW_bm		(1<<ADC_POW)
#define ADC_LOGIC_VAL	ADC_VOLTAGE.CH0RESL
#define ADC_POW_VAL		ADC_VOLTAGE.CH1RESL
#define ADC_LOGIC_FLAG	ADC_VOLTAGE.CH0.INTFLAGS
#define ADC_POW_FLAG	ADC_VOLTAGE.CH1.INTFLAGS
#define ADC_LOGIC_START	(ADC_VOLTAGE.CH0.CTRL = 1<<7 | 1)
#define ADC_POW_START	(ADC_VOLTAGE.CH1.CTRL = 1<<7 | 1)

#define PORT_THERM		PORTB
#define ADC_THERM		ADCB
#define THERM0			0
#define THERM1			1
#define THERM2			2
#define THERM0_bm		(1<<THERM0)
#define THERM1_bm		(1<<THERM1)
#define THERM2_bm		(1<<THERM2)
#define THERM0_VAL		ADC_THERM.CH0RESL
#define THERM1_VAL		ADC_THERM.CH1RESL
#define THERM2_VAL		ADC_THERM.CH2RESL
#define THERM0_FLAG		ADC_THERM.CH0.INTFLAGS
#define THERM1_FLAG		ADC_THERM.CH1.INTFLAGS
#define THERM2_FLAG		ADC_THERM.CH2.INTFLAGS
#define THERM0_START	(ADC_THERM.CH0.CTRL = 1<<7 | 1)
#define THERM1_START	(ADC_THERM.CH1.CTRL = 1<<7 | 1)
#define THERM2_START	(ADC_THERM.CH2.CTRL = 1<<7 | 1)

// RS422/485 - SSI
#define PORT_SSI0		PORTC
#define SSI0_USART		USARTC0
#define SSI0_DAT		2
#define SSI0_CLK		3
#define SSI0_DAT_bm		(1<<SSI0_DAT)
#define SSI0_CLK_bm		(1<<SSI0_CLK)

#define PORT_SSI1		PORTC
#define SSI1_USART		USARTC1
#define SSI1_SPI		SPIC
#define SSI1_DAT		6
#define SSI1_CLK		7
#define SSI1_DAT_bm		(1<<SSI1_DAT)
#define SSI1_CLK_bm		(1<<SSI1_CLK)

#define PORT_SSI2		PORTD
#define SSI2_USART		USARTD0
#define SSI2_DAT		2
#define SSI2_CLK		3
#define SSI2_DAT_bm		(1<<SSI2_DAT)
#define SSI2_CLK_bm		(1<<SSI2_CLK)

#define PORT_SSI3		PORTD
#define SSI3_USART		USARTD1
#define SSI3_SPI		SPID
#define SSI3_DAT		6
#define SSI3_CLK		7
#define SSI3_DAT_bm		(1<<SSI3_DAT)
#define SSI3_CLK_bm		(1<<SSI3_CLK)

// BiSS C
#define PORT_BISS		PORTD
#define BISS_SPI		SPID
#define BISS_DAT		6
#define BISS_CLK		7
#define BISS_DAT_bm		(1<<BISS_DAT)
#define BISS_CLK_bm		(1<<BISS_CLK)


// Time step timer
#define TC_STEP				TCC1
#define TC_STEP_OVF_vect	TCC1_OVF_vect

// Velocity step timer
#define TC_VEL				TCD1
#define TC_VEL_OVF_vect		TCD1_OVF_vect


// Amp controls
#define PORT_AMP_CTL		PORTD
#define TC_PWM				TCD0
#define HIRES_PWM			HIRESD
#define PWM					0
#define DIRECTION			1
#define PWM_bm				(1<<PWM)
#define DIRECTION_bm		(1<<DIRECTION)
//#define SetDirection(_dir)	( ((_dir) == 0)?(PORT_AMP_CTL.OUTCLR = DIRECTION_bm):(PORT_AMP_CTL.OUTSET = DIRECTION_bm))
#define PWMdis()			(TC_PWM.CTRLB	&= ~TC0_CCAEN_bm)


#define PORT_AMP_UART	PORTF
#define AMP_RS232		USARTF1
#define AMP_BAUD		1152

// Inc. Encoder:  NOTE, the encoder pins need to be consecutive and in the order A,B,Z
#define LINECOUNT			3786
#define ENC_TOP				(LINECOUNT*4-1)
#define PORT_ENCODER		PORTF
#define TC_ENC				TCF0
#define ENC_A				0
#define ENC_B				1
#define ENC_Z				2
#define ENC_A_bm			(1<<ENC_A)
#define ENC_B_bm			(1<<ENC_B)
#define ENC_Z_bm			(1<<ENC_Z)
#define ENC_EVSYS_CHMUX_A	EVSYS_CHMUX_PORTF_PIN0_gc
#define ENC_EVSYS_CHMUX_Z	EVSYS_CHMUX_PORTF_PIN2_gc





// global struct for the ISRs
typedef struct {

	uint8_t limits;				// what limit switches were hit
	uint8_t status;				// what caused the error
	uint8_t state;				// what state to be in
	uint8_t error_cnt			// error counter - used for debouncing the pannic

}	flags;

volatile flags global_flags;


#include "../libs/led.h"

//////////////////////////////
// Simple helper functions
// Some predefined LED colors

// indicates general error (or that many errors occurred)
inline void led_blink_red() {
	blinkLED();
	SetDutyR(0x7FFF);
	SetDutyG(0x000F);
	SetDutyB(0x000F);
}

// indicates bad a motor
inline void led_blink_dark_purple() {
	blinkLED();
	SetDutyR(0x0FFF);
	SetDutyG(0x00FF);
	SetDutyB(0x07FF);
}

// indicates problem with encoder
inline void led_blink_yellow() {
	blinkLED();
	SetDutyR(0x7FFF);
	SetDutyG(0x5FFF);
	SetDutyB(0x00FF);
}

// limit switch hit
inline void led_blink_orange() {
	blinkLED();
	SetDutyR(0x7FFF);
	SetDutyG(0x2FFF);
	SetDutyB(0x000F);
}

// timer overflow
inline void led_blink_blue() {
	blinkLED();
	SetDutyR(0x000F);
	SetDutyG(0x000F);
	SetDutyB(0x0FFF);
}


inline void led_blink_bluegreen() {
	blinkLED();
	SetDutyR(0x000F);
	SetDutyG(0x7FFF);
	SetDutyB(0x07FF);
}


// waiting for the panic line
inline void led_solid_red() {
	solidLED();
	SetDutyR(0x7FFF);
	SetDutyG(0x000F);
	SetDutyB(0x000F);
}

// opening the leg
inline void led_solid_orange() {
	solidLED();
	SetDutyR(0x7FFF);
	SetDutyG(0x0FFF);
	SetDutyB(0x000F);
}


// we're idle and not in op mode
inline void led_solid_purple() {
	solidLED();
	SetDutyR( 0x7FF0 );
	SetDutyG( 0x00FF );
	SetDutyB( 0x0FFF );
}

// disabled
inline void led_solid_white() {
	solidLED();
	SetDutyR(0x7FFF);
	SetDutyG(0x7FFF);
	SetDutyB(0x2FFF);
}

inline void led_solid_bluegreen() {
	solidLED();
	SetDutyR(0x000F);
	SetDutyG(0x7FFF);
	SetDutyB(0x07FF);
}


#endif // MENIAL_IO_H





