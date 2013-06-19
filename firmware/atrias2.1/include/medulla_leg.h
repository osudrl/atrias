#ifndef MEDULLA_LEG_H
#define MEDULLA_LEG_H

#include <stdio.h>

// Include the robot definitions
#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"

// Include medulla_lib headers
#include "ethercat.h"
#include "limit_switch.h"
#include "adc.h"
#include "biss_encoder.h"
#include "quadrature_encoder.h"
#include "adc124.h"

#include "amplifier.h"

// Defines for amplifier error output
#define AMP_DIO_PORT PORTH
#define AMP1_ERROR_bm (1<<0)
#define AMP1_ENABLED_bm (1<<1)
#define AMP2_ERROR_bm (1<<4)
#define AMP2_ENABLED_bm (1<<5)

// Defines for which systems to check for error state
#define ERROR_CHECK_LIMIT_SWITCH
#define ERROR_CHECK_THERMISTORS
//#define ERROR_CHECK_MOTOR_VOLTAGE
//#define ERROR_CHECK_LOGIC_VOLTAGE
//#define ERROR_CHECK_ENCODER
//#define ERROR_CHECK_AMP
#define MEDULLA_USE_HALT 

#define LOC_TO_COUNTS(POS, CALIB_LOC, CALIB_VAL, RAD_PER_TICK) \
	((uint32_t)((((POS)-(CALIB_LOC))/(RAD_PER_TICK)) + CALIB_VAL))

#define DAMPING_GAIN 60				// Gain used for error damping mode, this is in the same units as the controller computer.
#define DAMPING_GAIN_CONSTANT 3	// This constant scales the above gain so that it can be used on incremental encoder values and PWM values.
#define MOD(A,B) ((((A) % (B)) + (B)) % (B))
#define DAMPING_CURRENT_LIMIT 19900
#define DAMPING_MAX_CURRENT 60
#define DAMPING_DISTANCE_CONSTANT (1.0/24824.0)

void leg_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog);

void leg_enable_outputs(void);
void leg_disable_outputs(void);
void leg_update_inputs(uint8_t id);  /**< Function called to read all the sensors */
bool leg_run_halt(uint8_t id);       /**< Runs the halt controller. Returns true if the controller wants to continue running */ 
void leg_update_outputs(uint8_t id); /**< Called to update motor outputs */
void leg_estop(void);                /**< Called to when estop happens, should send 0 torques to motor */
bool leg_check_error(uint8_t id);    /**< Checks sensor readings to decide if estop should happen, returns true if there should be an error */
bool leg_check_halt(uint8_t id);     /**< Checks sensors to determine if medulla should go into halt state, returns true if medulla should go into halt */
void leg_reset_error();
void leg_wait_loop();

#endif //MEDULLA_LEG_H
