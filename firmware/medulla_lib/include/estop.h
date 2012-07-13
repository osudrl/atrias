#ifndef ESTOP_H
#define ESTOP_H

/** @file
 *  @brief EStop libaray for medulla
 *
 *  This library extends and wraps the limit switch library to provide a simple
 *  estop libary. This library assumes the estop system uses two IO lines. The
 *  Panic pin is an output, which the xmega drives high, when it wants to signal
 *  an estop, it sets the pin low. The EStopped line is an input which is
 *  normally low and goes high when an e-stop condition occures.
 */

#include <avr/io.h>
#include <stdbool.h>

#include "io_pin.h"
#include "limit_switch.h"

/** @brief struct for keeping track of the estop pins
 */
typedef struct {
	io_pin_t panic_pin;
	io_pin_t estop_pin;
	limit_sw_port_t limit_sw_port;
} estop_port_t;

#define ESTOP_USES_PORT(PORT) \
ISR(PORT##_INT0_vect) { \
        /* First reset the timer value*/ \
        ((TC1_t*)(_limit_sw_##PORT->counter_pntr))->CNT = 0; \
        \
        /* If one of the switches is high, start the counter if it's not already started */ \
        if (_limit_sw_##PORT->limit_sw_port->IN & _limit_sw_##PORT->pin_mask) \
                ((TC1_t*)(_limit_sw_##PORT->counter_pntr))->CTRLA = TC_CLKSEL_DIV4_gc; \
} \

#define ESTOP_USES_COUNTER(COUNTER) \
ISR(COUNTER##_OVF_vect) { \
        /* If we overflowed, then we disable the timer */ \
        ((TC1_t*)(_limit_sw_##COUNTER->counter_pntr))->CTRLA = TC_CLKSEL_OFF_gc; \
        \
        /* Now we will check if the limit switch is triggered */ \
        if (_limit_sw_##COUNTER->limit_sw_port->IN & _limit_sw_##COUNTER->pin_mask) \
                /* If it is still triggered, then call the callback function */ \
                _limit_sw_##COUNTER->callback(); \
} \

estop_port_t estop_init_port(io_pin_t panic_pin, io_pin_t estop_pin, void *counter_pointer, void (*callback)(void));

int estop_enable_port(estop_port_t *estop_port);

void estop_disable_port(estop_port_t *estop_port);

void estop_assert_port(estop_port_t *estop_port);

void estop_deassert_port(estop_port_t *estop_port);

bool estop_is_estopped(estop_port_t *estop_port);

#endif // ESTOP_H
