#ifndef MEDULLA_H
#define MEDULLA_H

#include <stdio.h>

// Include AVR specific stuff
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Include the robot definitions
#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"

// Include medulla_lib definitions
#include "cpu.h"
#include "io_pin.h"
#include "uart.h"
#include "ethercat.h"

// Include headers for individual medullas
#include "imu.h"

// Watchdog Timer
#define WATCHDOG_TIMER TCE1
#define WATCHDOG_TIMER_RESET TCE1.CNT = 0

// Status LED definitions
#define LED_PORT PORTC
#define LED_MASK 0b111
#define LED_RED 0b001
#define LED_GREEN 0b010
#define LED_BLUE 0b100
#define LED_CYAN (LED_GREEN | LED_BLUE)
#define LED_VIOLATE (LED_RED | LED_BLUE)
#define LED_YELLOW (LED_RED | LED_GREEN)
#define LED_WHITE (LED_RED | LED_GREEN | LED_BLUE)

#define ENABLE_LEDS

// Defitions for ethercat communications
// Find the largest tx and rx sync manager sizes. This is used for allocating buffers.
#if (MEDULLA_LEG_INPUTS_SIZE < MEDULLA_HIP_INPUTS_SIZE)
	#if (MEDULLA_HIP_INPUTS_SIZE < MEDULLA_BOOM_INPUTS_SIZE)
		#define MAX_TX_SM_SIZE MEDULLA_BOOM_INPUTS_SIZE
	#else
		#define MAX_TX_SM_SIZE MEDULLA_HIP_INPUTS_SIZE
	#endif
#else 
	#if (MEDULLA_LEG_INPUTS_SIZE < MEDULLA_BOOM_INPUTS_SIZE)
		#define MAX_TX_SM_SIZE MEDULLA_BOOM_INPUTS_SIZE
	#else
		#define MAX_TX_SM_SIZE MEDULLA_LEG_INPUTS_SIZE
	#endif
#endif

#if (MEDULLA_LEG_OUTPUTS_SIZE < MEDULLA_HIP_OUTPUTS_SIZE)
	#if (MEDULLA_HIP_OUTPUTS_SIZE < MEDULLA_BOOM_OUTPUTS_SIZE)
		#define MAX_RX_SM_SIZE MEDULLA_BOOM_OUTPUTS_SIZE
	#else
		#define MAX_RX_SM_SIZE MEDULLA_HIP_OUTPUTS_SIZE
	#endif
#else 
	#if (MEDULLA_LEG_OUTPUTS_SIZE < MEDULLA_BOOM_OUTPUTS_SIZE)
		#define MAX_RX_SM_SIZE MEDULLA_BOOM_OUTPUTS_SIZE
	#else
		#define MAX_RX_SM_SIZE MEDULLA_LEG_OUTPUTS_SIZE
	#endif
#endif

ecat_slave_t ecat_port;
uint8_t ecat_tx_sm_buffer[MAX_TX_SM_SIZE]; ///< Buffer used for the tx sync manager 
uint8_t ecat_rx_sm_buffer[MAX_RX_SM_SIZE]; ///< Buffer used for the rx sync manager
medulla_state_t *commanded_state;
medulla_state_t *current_state;
uint8_t *packet_counter;

// Definitions for the medulla ID dip switches
#define MEDULLA_ID_PORT PORTJ
#define MEDULLA_ID_MASK 0x3F
uint8_t medulla_id;

// Timestamp definitions
#define TIMESTAMP_COUNTER TCC0

#endif //MEDULLA_H
