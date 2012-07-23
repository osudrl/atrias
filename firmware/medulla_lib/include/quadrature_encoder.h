#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

/** @file
 *  @brief Driver to interface with an quadriture incremental encoder
 *
 *  Using the counters and event system in the xMega it is possible to
 *  autonomously keep track of the encoder position. This driver provides an
 *  interface to configure the xmega for this type of operation. It also
 *  provides functionality to read the current encoder posision. This driver
 *  uses event channel 0 for the quadrature signal decoding. If the index input
 *  is used, then event channel 1 is also used.
 */

#include <avr/io.h>
#include <stdbool.h>
#include "io_pin.h"

/** @brief Struct defining a quadrature encoder configuration.
 *
 *  This struct is used to store the configuration of the quadrature encoder as
 *  well as to keep track of the encoder inside the users program.
 */ 
typedef struct {
	io_pin_t quadrature_base_pin; /**< IO pin of the first pin used for quadrature input */
	io_pin_t index_pin;           /**< Pin that the index input is conencted to */
	bool uses_index;              /**< Stores if the index pin is being used */
	TC0_t *counter_reg;           /**< Pointer to the counter register being used, this could also point to a TC1_t register */
	uint16_t lines_per_rev;      /**< Lines per revolution of the encoder */
} quadrature_encoder_t;

/** @brief Sets up xmega hardware for quadrature encoder decoding
 *
 *  This function creates a quadrature encoder struct with the encoder
 *  configuration. It also configures the xMega's counters and event system to
 *  decode the quadrature signal from the encoder. Because the xMega event
 *  system is decode the encoder, the two quadrature inputs from the encoder
 *  must be sequential pins on the same port.
 *
 *  @param quadrature_base_pin Pin of the first quadrature input.
 *  @param index_pin IO pin that the index output is connected to.
 *  @param uses_index True if the index pin should be used to reset the timer.
 *  @param counter_reg Pointer a TC0_t or TC1_t register to use for counting.
 *  @param lines_per_rev Number of encoder counts per revolution.
 *  @return quadrature_encoder_t striuct to keep track of the configuration.
 */ 
quadrature_encoder_t quadrature_encoder_init(io_pin_t quadrature_base_pin, io_pin_t index_pin, bool uses_index, void *counter_reg, uint16_t lines_per_rev);

/** @brief Gets the current encoder position from the counter
 *
 *  @param encoder pointer to the encoder struct to get the position for.
 *  @return 16 bit position of the encoder in encoder coutns.
 */
uint16_t quadrature_encoder_get_value(quadrature_encoder_t *encoder);

#endif //QUADRATURE_ENCODER_H
