#ifndef PWM_H
#define PWM_H

/** @file
 *  @brief This file provides a PWM driver for the xMega
 */

#include <avr/io.h>
#include "io_pin.h"

typedef enum {
	pwm_div1 = TC_CLKSEL_DIV1_gc,
	pwm_div2 = TC_CLKSEL_DIV2_gc,
	pwm_div4 = TC_CLKSEL_DIV4_gc,
	pwm_div8 = TC_CLKSEL_DIV8_gc,
	pwm_div64 = TC_CLKSEL_DIV64_gc,
	pwm_div256 = TC_CLKSEL_DIV256_gc,
	pwm_div1024 = TC_CLKSEL_DIV1024_gc
} pwm_clk_div_t;

typedef struct {
	io_pin_t pwm_pin;		/**< Pin to ouput PWM on */
	TC0_t *tc0_register;		/**< Pointer to timer counter register if it uses a TC0 Counter */
	TC1_t *tc1_register;		/**< Pointer to the timer counter register if it uses a TC1 counter */
	uint16_t *cc_register;		/**< Pointer to the capture compare register. This is used to set PWM value;*/
	pwm_clk_div_t clock_divider;	/** Desired clock divider */
} pwm_output_t;

/** @brief Configures the timer counter for generating PWM
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
