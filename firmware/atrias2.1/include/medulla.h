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
#include "estop.h"
#include "uart.h"
#include "ethercat.h"
#include "ad7193.h"

// Include headers for individual medullas
#include "medulla_leg.h"
#include "medulla_boom.h"
#include "medulla_hip.h"
#include "medulla_imu.h"

#include "amplifier.h"

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

// Debug uart port definitions
#define DEBUG_UART_TX_BUFFER_SIZE 1024
#define DEBUG_UART_RX_BUFFER_SIZE 16
uint8_t debug_uart_tx_buffer[DEBUG_UART_TX_BUFFER_SIZE];
uint8_t debug_uart_rx_buffer[DEBUG_UART_RX_BUFFER_SIZE];
uart_port_t debug_port;

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

// Master watchdog counter definitions
uint16_t *master_watchdog_counter;
uint16_t previous_master_watchdog;
uint8_t master_watchdog_errors;

// Estop
estop_port_t estop_port; /** @brief struct used by the estop driver */

// Timestamp definitions
#define TIMESTAMP_COUNTER TCC0

// EStop timeout definitions
#define ESTOP_TIMEOUT_LENGTH 1000
uint16_t estop_timeout_counter;

void main_estop();
void amplifier_debug();
void imu_debug();

// Function Pointers to the individual medulla functions
/** @brief Function called to initilize hardware
 *
 *  This function should setup the hardware of the robot. It also needs to call
 *  the sm and pdo entry configuration functions on the given ethercat struct.  
 *  
 *  @param ecat_slave Pointer to the ethercat slave struct to configure
 *  @param commanded_state pointer to the ecat master commanded state poitner
 *  @param commanded_state pointer to the current medulla state pointer
 *  @param tx_sm_buffer Pointer to the buffer to use for the tx sm
 *  @param rx_sm_buffer Pointer to the buffer to use for the rx sm
 *  @param id The 6 bit id of the medulla
 *  @param timestamp_timer Timer to use for generating timestamps
 */
void (*initilize)(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer,  medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdg); 

void (*enable_outputs)(void);
void (*disable_outputs)();
void (*update_inputs)(uint8_t id);  /**< Function called to read all the sensors */
bool (*run_halt)(uint8_t id);       /**< Runs the halt controller. Returns true if the controller wants to continue running */
void (*update_outputs)(uint8_t id); /**< Called to update motor outputs */
void (*estop)(void);                /**< Called to when estop happens, should send 0 torques to motor */
bool (*check_error)(uint8_t id);    /**< Checks sensor readings to decide if estop should happen, returns true if there should be an error */
bool (*check_halt)(uint8_t id);     /**< Checks sensors to determine if medulla should go into halt state, returns true if medulla should go into halt */
void (*reset_error)(void);
void (*wait_loop)(void);

#endif //MEDULLA_H
