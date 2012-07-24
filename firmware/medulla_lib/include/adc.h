#ifndef ADC_H
#define ADC_H

/** @file
 *  @brief Provides a asyncronous interface to the xMega internal ADCs
 *  
 *  This driver provides an interrupt driven ADC interface which allows the user
 *  to start the ADC read cycle and all the ADCs will be read sequentially. The
 *  values from the ADC will get written into locations of the user's choosing.
 *  At a later time the user can then check if the read was complete.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>


/** @brief This struct stores the ADC configuration
 */
typedef struct {
	ADC_t *adc_port;
	volatile uint8_t pin_mask;
	uint16_t* adc_output_pointers[8];
} adc_port_t;

adc_port_t *_adc_port_ADCA,
           *_adc_port_ADCB;


#define ADC_USES_PORT(ADC) \
ISR(ADC##_CH0_vect, ISR_NOBLOCK) { \
	PORTC.OUTSET = 1;\
	if (_adc_port_##ADC->pin_mask & 0b00000001) { \
		/* We just read pin 0 */ \
		*(_adc_port_##ADC->adc_output_pointers[0]) = ADC.CH0.RES; \
		_adc_port_##ADC->pin_mask &= 0b11111110; \
		/* Check if we need to start the next pin */ \
		if (_adc_port_##ADC->pin_mask & 0b00010000) { \
			ADC.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc; \
			ADC.CH0.CTRL |= ADC_CH_START_bm; \
		} \
	} \
	else { \
		/* we just read pin 4 */ \
		*(_adc_port_##ADC->adc_output_pointers[4]) = ADC.CH0.RES; \
		_adc_port_##ADC->pin_mask &= 0b11101111; \
	} \
	PORTC.OUTCLR = 1;\
} \
ISR(ADC##_CH1_vect,ISR_NOBLOCK) { \
	PORTC.OUTSET = 1;\
	if (_adc_port_##ADC->pin_mask & 0b00000010) { \
		/* We just read pin 1 */ \
		*(_adc_port_##ADC->adc_output_pointers[1]) = ADC.CH1.RES; \
		_adc_port_##ADC->pin_mask &= 0b11111101; \
		/* Check if we need to start the next pin */ \
		if (_adc_port_##ADC->pin_mask & 0b00100000) { \
			ADC.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc; \
			ADC.CH1.CTRL |= ADC_CH_START_bm; \
		} \
	} \
	else { \
		/* we just read pin 5 */ \
		*(_adc_port_##ADC->adc_output_pointers[5]) = ADC.CH1.RES; \
		_adc_port_##ADC->pin_mask &= 0b11011111; \
	} \
	PORTC.OUTCLR = 1;\
} \
ISR(ADC##_CH2_vect,ISR_NOBLOCK) { \
	PORTC.OUTSET = 1;\
	if (_adc_port_##ADC->pin_mask & 0b00000100) { \
		/* We just read pin 0 */ \
		*(_adc_port_##ADC->adc_output_pointers[2]) = ADC.CH2.RES; \
		_adc_port_##ADC->pin_mask &= 0b11111011; \
		/* Check if we need to start the next channel */ \
		if (_adc_port_##ADC->pin_mask & 0b01000000) { \
			ADC.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc; \
			ADC.CH2.CTRL |= ADC_CH_START_bm; \
		} \
	} \
	else { \
		/* we just read pin 4 */ \
		*(_adc_port_##ADC->adc_output_pointers[6]) = ADC.CH2.RES; \
		_adc_port_##ADC->pin_mask &= 0b10111111; \
	} \
	PORTC.OUTCLR = 1;\
} \
ISR(ADC##_CH3_vect, ISR_NOBLOCK) { \
	PORTC.OUTSET = 1;\
	if (_adc_port_##ADC->pin_mask & 0b00001000) { \
		/* We just read pin 0 */ \
		*(_adc_port_##ADC->adc_output_pointers[3]) = ADC.CH3.RES; \
		_adc_port_##ADC->pin_mask &= 0b11110111; \
		/* Check if we need to start the next channel */ \
		if (_adc_port_##ADC->pin_mask & 0b10000000) { \
			ADC.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN7_gc; \
			ADC.CH3.CTRL |= ADC_CH_START_bm; \
		} \
	} \
	else { \
		/* we just read pin 4 */ \
		*(_adc_port_##ADC->adc_output_pointers[7]) = ADC.CH3.RES; \
		_adc_port_##ADC->pin_mask &= 0b01111111; \
	} \
	PORTC.OUTCLR = 1;\
} \

adc_port_t adc_init_port(ADC_t *adc);
void adc_connect_port(adc_port_t *adc);
void adc_init_pin(adc_port_t *adc, uint8_t pin, uint16_t *pin_data);
void adc_start_read(adc_port_t *adc, uint8_t pin_mask);
bool adc_read_complete(adc_port_t *adc);

#endif //ADC_H
