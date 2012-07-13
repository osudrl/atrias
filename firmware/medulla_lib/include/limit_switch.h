#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

/** @file
 *  @brief This library provides an interrupt driven limit switch driver on a single port.
 *
 *  This driver uses the pin change interrupt on a given port to detect when any
 *  limit switch is triggered. It is assumed that the conencted limit switches
 *  are normally closed. This library enables the pull up resistors on all the
 *  switches, and the switch is said to be triggered when a pin is high. When a
 *  limit switch is pressed, a function pointer is called, so the user can run
 *  custom code. The interrupt used by this function is a high level interrupt
 * 
 *  Debouncing is done using a timer counter. This timer does not have to be on
 *  the same port as the limit swites.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

/** @brief Struct for storing a port's limit switch configuration
 */
typedef struct {
	PORT_t *limit_sw_port;	/**< Pointer to the port register */
	uint8_t pin_mask;	/**< Bitmask that specifies the pins to use */
	void *counter_pntr;	/**< Pointer to the timer counter for debouncing. A void* is used so it can point to either a TC0_t or a TC1_t struct */
	void (*callback)(void);	/**< Function pointer to the function called when a limit switch is pressed */
	bool waiting_timeout;	/**< True when waiting for a timeout to occure before a new trigger can happen */
} limit_sw_port_t;

limit_sw_port_t *_limit_sw_PORTA, /**<Pointer to port A limit switch port struct */
                *_limit_sw_PORTB, /**<Pointer to port B limit switch port struct */
                *_limit_sw_PORTC, /**<Pointer to port C limit switch port struct */
                *_limit_sw_PORTD, /**<Pointer to port D limit switch port struct */
                *_limit_sw_PORTE, /**<Pointer to port E limit switch port struct */
                *_limit_sw_PORTF, /**<Pointer to port F limit switch port struct */
                *_limit_sw_PORTH, /**<Pointer to port H limit switch port struct */
                *_limit_sw_PORTJ, /**<Pointer to port J limit switch port struct */
                *_limit_sw_PORTK; /**<Pointer to port K limit switch port struct */

limit_sw_port_t *_limit_sw_TCC0, /**< Pointer to timer counter C0 limit switch struct */
                *_limit_sw_TCC1, /**< Pointer to timer counter C1 limit switch struct */
                *_limit_sw_TCD0, /**< Pointer to timer counter D0 limit switch struct */
                *_limit_sw_TCD1, /**< Pointer to timer counter D1 limit switch struct */
                *_limit_sw_TCE0, /**< Pointer to timer counter E0 limit switch struct */
                *_limit_sw_TCE1, /**< Pointer to timer counter E1 limit switch struct */
                *_limit_sw_TCF0, /**< Pointer to timer counter F0 limit switch struct */
                *_limit_sw_TCF1; /**< Pointer to timer counter F1 limit switch struct */

#define LIMIT_SW_USES_PORT(PORT) \
ISR(PORT##_INT0_vect) { \
	/* First reset the timer value*/ \
	((TC1_t*)(_limit_sw_##PORT->counter_pntr))->CNT = 0; \
	\
	/* If one of the switches is high, start the counter if it's not already started */ \
	if (_limit_sw_##PORT->limit_sw_port->IN & _limit_sw_##PORT->pin_mask) \
		((TC1_t*)(_limit_sw_##PORT->counter_pntr))->CTRLA = TC_CLKSEL_DIV4_gc; \
} \

#define LIMIT_SW_USES_COUNTER(COUNTER) \
ISR(COUNTER##_OVF_vect) { \
	/* If we overflowed, then we disable the timer */ \
	((TC1_t*)(_limit_sw_##COUNTER->counter_pntr))->CTRLA = TC_CLKSEL_OFF_gc; \
	\
	/* Now we will check if the limit switch is triggered */ \
	if (_limit_sw_##COUNTER->limit_sw_port->IN & _limit_sw_##COUNTER->pin_mask) \
		/* If it is still triggered, then call the callback function */ \
		_limit_sw_##COUNTER->callback(); \
} \


/** @brief Initilizes a port containing limit switches
 */
limit_sw_port_t limit_sw_init_port(PORT_t *limit_sw_port, uint8_t pin_mask, void *counter_pntr, void (*callback)(void));

/** @brief Enables limit switch interrupt
 *  @returns Returns 0 upon sucess, -1 if something is already using this interrupt
 */
int limit_sw_enable_port(limit_sw_port_t *limit_sw_port);

/** @brief Disalbes limit switch interrupts on a port
 */
void limit_sw_disable_port(limit_sw_port_t *limit_sw_port);

/** @brief Get the current values of all the limit switches
 */
uint8_t limit_sw_get_port(limit_sw_port_t *limit_sw_port);

#endif //LIMIT_SWITCH_H

