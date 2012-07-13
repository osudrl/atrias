#ifndef PWM_H
#define PWM_H

/** @file
 *  @brief This file provides a PWM driver for the xMega
 *
 *  This library uses one of the timer counters in the xMega to generate PWM on
 *  one of the IO pins. Only pins 0-5 on ports C, D, E, and F can be used to
 *  generate PWM, as these are the only pins that are connected to timer
 *  counters. Since there are multiple pins are connected to a single timer, it
 *  is important to realize that all the pins connected to a single timer share
 *  the same timer configuration. Whenever the pwm_initilize_output() function
 *  is called the configuration for that pin's timer is overwritten, and any
 *  currently running PWM outputs will be reconfigured. 
 */

#include <avr/io.h>
#include "io_pin.h"

/** @brief Type for defining clock divider
 *
 *  This enum is used by pwm_initilize_output to set the clock divider for the
 *  timer counter. This enum uses values that can be direcly put into the CTRLA
 *  register to set the clock divider.
 */
typedef enum {
	pwm_div1 = TC_CLKSEL_DIV1_gc,
	pwm_div2 = TC_CLKSEL_DIV2_gc,
	pwm_div4 = TC_CLKSEL_DIV4_gc,
	pwm_div8 = TC_CLKSEL_DIV8_gc,
	pwm_div64 = TC_CLKSEL_DIV64_gc,
	pwm_div256 = TC_CLKSEL_DIV256_gc,
	pwm_div1024 = TC_CLKSEL_DIV1024_gc
} pwm_clk_div_t;

/** @brief Struct used to store informatino about a PWM output.
 *
 *  This struct defines all the need information about a particular PWM output.
 *  The struct contains pointers to both a TC0_t struct and a TC1_t struct.
 *  Depending on which pin is being used, either a TC0 and a TC1 might be
 *  being used generate the PWM. If a TCx0 is being used, tc1_register is set to
 *  zero, if a TCx1 is being used, then tc0_register is set to zero.
 */
typedef struct {
	io_pin_t pwm_pin;		/**< Pin to ouput PWM on */
	TC0_t *tc0_register;		/**< Pointer to timer counter register if it uses a TC0 Counter */
	TC1_t *tc1_register;		/**< Pointer to the timer counter register if it uses a TC1 counter */
	uint16_t *cc_register;		/**< Pointer to the capture compare register. This is used to set PWM value;*/
	pwm_clk_div_t clock_divider;	/**< Desired clock divider */
} pwm_output_t;

/** @brief Configures the timer counter for generating PWM
 *
 *  This function creates a pwm_output_t struct
 *
 *  @note Setting the clock_devider and maximum counter value will change these
 *  values for all PWM outputs using this counters.
 */
pwm_output_t pwm_initilize_output(io_pin_t pwm_pin, pwm_clk_div_t clock_divider, uint16_t cnt_max);

/** @brief Enables the PWM generation output
 *
 *  @note Calling this function on a PWM port that is on an already running
 *  counter will switch the clock divider value to the PWM output's counter. Any
 *  other PWM output using this counter will also be changed.
 */
void pwm_enable_output(pwm_output_t *pwm_output);

/** @brief 
 *
 *  @note This stops the timer counter, all PWM outputs that use this counter
 *  will stop.
 */
void pwm_disable_output(pwm_output_t *pwm_output);

/** @brief Sets the high time in counter ticks of the PWM signal
 *
 */
void pwm_set_output(pwm_output_t *pwm_ouput, uint16_t value);

#endif
