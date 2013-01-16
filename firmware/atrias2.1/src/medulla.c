#include "medulla.h"

//--- Define the interrupt functions --- //
// Debug port
UART_USES_PORT(USARTE0)

//The ESTOP is on port J and uses TCE0 as it's debounce timer
ESTOP_USES_PORT(PORTJ)
ESTOP_USES_COUNTER(TCE0)

// Ethercat on port E
ECAT_USES_PORT(SPIE);

// Interrupt for handling watchdog (we don't need a driver for this)
ISR(TCE1_OVF_vect) {
	WATCHDOG_TIMER.INTCTRLA = TC_OVFINTLVL_OFF_gc;
	estop();
	LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK);
	printf("[ERROR] Watchdog timer overflow\n");
	while(1);
}

// Limit Switches
LIMIT_SW_USES_PORT(PORTK)
LIMIT_SW_USES_COUNTER(TCF0)

// BISS and SSI encoders use the SPI ports
SPI_USES_PORT(SPIC)
SPI_USES_PORT(SPID)
SPI_USES_PORT(SPIF)

// Amplifier on port D0
UART_USES_PORT(USARTD0)

// ADCs on port a and b
ADC_USES_PORT(ADCA)
ADC_USES_PORT(ADCB)
USART_ADC_USES_PORT(USARTF0)

int main(void) {
	// Initilize the clock to 32 Mhz oscillator
	if(cpu_set_clock_source(cpu_32mhz_clock) == false) {
		PORTC.DIRSET = 1;
		PORTC.OUTSET = 1;
	}

	// Configure and enable all the interrupts
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
	sei();

	// Initilize the LED port
	LED_PORT.DIRSET = LED_MASK;

	// Initilize the id DIP switches
	PORTCFG.MPCMASK =  MEDULLA_ID_MASK;
	MEDULLA_ID_PORT.PIN0CTRL = PORT_OPC_PULLUP_gc;
	_delay_ms(1);
	medulla_id = MEDULLA_ID_PORT.IN & MEDULLA_ID_MASK;

	// Check if we are in the amplifier debug mode
	if (medulla_id == MEDULLA_AMPLIFIER_DEBUG) {
		amplifier_debug();
	}

	// Initilize the debug uart
	debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, debug_uart_tx_buffer, DEBUG_UART_TX_BUFFER_SIZE, debug_uart_rx_buffer, DEBUG_UART_RX_BUFFER_SIZE);

	// Don't enable this port if this is a right hip medulla, because that will interfere with the IMU communication
	if (medulla_id != MEDULLA_RIGHT_HIP_ID)
		uart_connect_port(&debug_port, true);

	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla] Initilizing Medulla\n");
	#endif

	// Initilizing EStop
	#ifdef DEBUG_HIGH
	printf("[Medulla] Initilizing E-Stop\n");
	#endif
	estop_port = estop_init_port(io_init_pin(&PORTJ,6),io_init_pin(&PORTJ,7),&TCE0,main_estop);

	// Initilizing timestamp counter
	#ifdef DEBUG_HIGH
	printf("[Medulla] Initilizing timestamp counter\n");
	#endif
	TIMESTAMP_COUNTER.CTRLA = TC_CLKSEL_DIV2_gc;

	// Initilize the EtherCAT
	#ifdef DEBUG_HIGH
	printf("[Medulla] Initilizing EtherCAT\n");
	#endif
	ecat_port = ecat_init_slave(&PORTE,&SPIE,io_init_pin(&PORTE,0),io_init_pin(&PORTE,1));
	// set the the IRQ pin so it sets the IRQ flags on the falling edge so we can check that for the DC clock
	PORTE.PIN1CTRL = PORT_ISC_FALLING_gc;
	PORTE.INT0MASK = 0b10;

//	medulla_id = 1;
	#if defined DEBUG_HIGH || defined DEBUG_LOW
	printf("[Medulla] Medulla ID is: %02x, ", medulla_id);
	#endif
	// Set up the function pointers for this medulla
	switch (medulla_id & MEDULLA_ID_PREFIX_MASK) {
		case MEDULLA_LEG_ID_PREFIX:
			#if defined DEBUG_HIGH || defined DEBUG_LOW
			printf("loading leg medulla.\n");
			#endif
			initilize = leg_initilize;
			enable_outputs = leg_enable_outputs;
			disable_outputs = leg_disable_outputs;
			update_inputs = leg_update_inputs;
			run_halt = leg_run_halt;
			update_outputs = leg_update_outputs;
			estop = leg_estop;
			check_error = leg_check_error;
			check_halt = leg_check_halt;
			reset_error = leg_reset_error;
			wait_loop = leg_wait_loop;
			break;

		case MEDULLA_HIP_ID_PREFIX:
			#if defined DEBUG_HIGH || defined DEBUG_LOW
			printf("loading hip medulla.\n");
			#endif
			initilize = hip_initilize;
			enable_outputs = hip_enable_outputs;
			disable_outputs = hip_disable_outputs;
			update_inputs = hip_update_inputs;
			run_halt = hip_run_halt;
			update_outputs = hip_update_outputs;
			estop = hip_estop;
			check_error = hip_check_error;
			check_halt = hip_check_halt;
			reset_error = hip_reset_error;
			wait_loop = hip_wait_loop;
			break;

		case MEDULLA_BOOM_ID_PREFIX:
			#if defined DEBUG_HIGH || defined DEBUG_LOW
			printf("loading boom medulla.\n");
			#endif
			initilize = boom_initilize;
			enable_outputs = boom_enable_outputs;
			disable_outputs = boom_disable_outputs;
			update_inputs = boom_update_inputs;
			run_halt = boom_run_halt;
			update_outputs = boom_update_outputs;
			estop = boom_estop;
			check_error = boom_check_error;
			check_halt = boom_check_halt;
			reset_error = boom_reset_error;
			wait_loop = boom_wait_loop;
			break;

		case MEDULLA_TEST_ID_PREFIX:
			#if defined DEBUG_HIGH || defined DEBUG_LOW
			printf("loading test medulla.\n");
			#endif
			/// TODO: Add code to map function pointers to test medulla
			break;
		default:
			#if defined DEBUG_HIGH || defined DEBUG_LOW
			printf(" unknown medulla.\n");
			#endif
			printf("[ERROR] Unknown medulla ID: %04x, aborting.\n",medulla_id);
			while (1);
	}

	// Wait for a second so all the hardware can initilize first
	_delay_ms(1000);

	// Call the initilize function to initilize all the hardware for this medulla
	#ifdef DEBUG_HIGH
	printf("[Medulla] Calling init for specific medulla\n");
	#endif
	initilize(medulla_id, &ecat_port, ecat_tx_sm_buffer, ecat_rx_sm_buffer, &commanded_state, &current_state, &packet_counter, &TIMESTAMP_COUNTER, &master_watchdog_counter);
	
	#ifdef DEBUG_HIGH
	printf("[Medulla] Switching printf to low level interrupt\n");
	#endif
	USARTE0.CTRLA = USART_RXCINTLVL_LO_gc | USART_TXCINTLVL_LO_gc;
	
	#ifdef DEBUG_HIGH
	printf("[Medulla] Starting watchdog timer\n");
	#endif
	// Now that everything is set up, start the watchdog timer
	WATCHDOG_TIMER.INTCTRLA = TC_OVFINTLVL_HI_gc;
	WATCHDOG_TIMER.CTRLA = TC_CLKSEL_DIV4_gc;

	#ifdef DEBUG_HIGH
	printf("[Medulla] Enabling E-Stop\n");
	#endif
	// and enable the estop initilize the estop
	estop_enable_port(&estop_port);

	#if defined DEUBG_LOW || defined DEBUG_HIGH
	printf("[Medulla] Starting state machine\n");
	printf("[State Machine] Entering state: Idle\n");
	#endif
	
	previous_master_watchdog = *master_watchdog_counter;
	#ifdef ENABLE_LEDS
	LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_GREEN;
	#endif
	while(1) {
		// Check if there was a falling edge of the ethercat IRQ pin
		if (PORTE.INTFLAGS & PORT_INT0IF_bm) {
			TIMESTAMP_COUNTER.CNT = 0; // First thing after finding a falling clock edge, clear the timestamp counter.
			PORTE.INTFLAGS = PORT_INT0IF_bm; // Now that we noticed DC clock, clear the interrupt flag
			// This is the signal to read all the sensors and run the state mechine

			// Update the inputs
			update_inputs(medulla_id);

			// Increment the packet counter
			*packet_counter += 1;

			// Send the new sensor data to the ethercat slave
			ecat_write_tx_sm(&ecat_port);

			// Read new commands from the ethercat slave
			ecat_read_rx_sm(&ecat_port);
	
			// Run state machine
			if (*current_state == medulla_state_idle) {
				#ifdef ENABLE_LEDS
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_GREEN;
				#endif
				// We can only go into init state from here, when we received the run command.
				
				// We will continually deassert the estop line here, because it takes a long time for it to respond
				estop_deassert_port(&estop_port);

				if (*commanded_state == medulla_state_run) {
					*current_state = medulla_state_init;
					#if defined DEUBG_LOW || defined DEBUG_HIGH
					printf("[State Machine] Entering state: Init\n");
					#endif
				}
				continue;
			}
			if (*current_state == medulla_state_init) {
				#ifdef ENABLE_LEDS
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_CYAN;
				#endif
				// First clear all the error flags
				reset_error();
				master_watchdog_errors = 0;
				if (check_error(medulla_id) || check_halt(medulla_id) || estop_is_estopped(&estop_port)) {
					// Something is wrong, either a error or halt state was requested
					// So we will just go into error.
					
					if (estop_is_estopped(&estop_port))
						printf("[Medulla] E-Stop pressed\n");
					*current_state = medulla_state_error;
					estop_timeout_counter = 0;
					#if defined DEUBG_LOW || defined DEBUG_HIGH
					printf("[State Machine] Entering state: Error\n");
					#endif
				}
				else {
					// The robot is in a fit state to enter run, arm the robot
					enable_outputs();
					*current_state = medulla_state_run;
					#if defined DEUBG_LOW || defined DEBUG_HIGH
					printf("[State Machine] Entering state: Run\n");
					#endif
					continue;
				}
			}
			if (*current_state == medulla_state_run) {
				#ifdef ENABLE_LEDS
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_BLUE;
				#endif
				// Here the robot is armed and dangerous. We will first update the state based
				// upon the maser's command. Then we will override that based upon hardware errors
				
				if ((*commanded_state == medulla_state_stop) || (*commanded_state == medulla_state_idle))
					*current_state = medulla_state_stop;
				else if (*commanded_state == medulla_state_halt)
					*current_state = medulla_state_halt;
				else if (*commanded_state == medulla_state_error) {
					*current_state = medulla_state_error;
					estop_timeout_counter = 0;
				}

				// Check the master watchdog counter
				if (*master_watchdog_counter == previous_master_watchdog)
					master_watchdog_errors += 1;
				else
					master_watchdog_errors = 0;

				previous_master_watchdog = *master_watchdog_counter;

				if (master_watchdog_errors >= 10) {
					#ifdef DEBUG_HIGH
					printf("[Medulla] Master watchdog error.\n");
					#endif
					*current_state = medulla_state_error;
				}

				// Now we check for errors based upon hardware
				if (check_halt(medulla_id))
					// The halt state was requested based upon hardware, so switch to that next
					*current_state = medulla_state_halt;

				if (check_error(medulla_id) || estop_is_estopped(&estop_port)) {
					// Either there was an error worthy hardware problem or the estop was asserted
					// Go into the error state
					if (estop_is_estopped(&estop_port))
						printf("[Medulla] E-Stop pressed\n");
					*current_state = medulla_state_error;
					estop_timeout_counter = 0;
					disable_outputs();
				}

				// If at this point we are still in the run state, then update the motors, and continue
				if (*current_state == medulla_state_run) {
					update_outputs(medulla_id);
					continue;
				}

				#if defined DEUBG_LOW || defined DEBUG_HIGH
				if (*current_state == medulla_state_stop)
					printf("[State Machine] Entering state: Stop\n");
				if (*current_state == medulla_state_halt)
					printf("[State Machine] Entering state: Halt\n");
				if (*current_state == medulla_state_error)
					printf("[State Machine] Entering state: Error\n");
				#endif			
				
			}
			if (*current_state == medulla_state_stop) {
				#ifdef ENABLE_LEDS
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_VIOLATE;
				#endif
				// In the stop state, the only place we can go, is back to idle
				// So we disable the outputs and return to idle
				disable_outputs();
				*current_state = medulla_state_idle;
				#if defined DEUBG_LOW || defined DEBUG_HIGH
				printf("[State Machine] Entering state: Idle\n");
				#endif
				continue;
			}
			if (*current_state == medulla_state_halt) {
				#ifdef ENABLE_LEDS
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_YELLOW;
				#endif
				// In the halt state, we can only go into error from here, so check if we want to go into error
				if (check_error(medulla_id) || estop_is_estopped(&estop_port)) {
					*current_state = medulla_state_error;
					disable_outputs();
					estop_timeout_counter = 0;
					#if defined DEUBG_LOW || defined DEBUG_HIGH
					printf("[State Machine] Entering state: Error\n");
					#endif

				}

				// If we are still in halt state, after this check, then run the halt controller and continue to next loop
				if (*current_state == medulla_state_halt) {
					if (!run_halt(medulla_id)) {
						*current_state = medulla_state_error;
						disable_outputs();
						estop_timeout_counter = 0;
					}
					#if defined DEUBG_LOW || defined DEBUG_HIGH
					if (*current_state == medulla_state_error)
						printf("[State Machine] Entering state: Error\n");
					#endif
					continue;
				}

				
			}
			if (*current_state == medulla_state_reset) {
				#ifdef ENABLE_LEDS
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_WHITE;
				#endif
				// This state let's us go back to idle after an error state.
				// First though, we have to deassert our estop line
				estop_deassert_port(&estop_port);
				reset_error();
				master_watchdog_errors = 0;
				*current_state = medulla_state_idle;
				#if defined DEUBG_LOW || defined DEBUG_HIGH
				printf("[State Machine] Entering state: Idle\n");
				#endif
				continue;
			}
			if ((*current_state < 0) || (*current_state == medulla_state_error) || (*current_state > 6)) { // current_state is either error or some invalid value
				main_estop();
				#ifdef ENABLE_LEDS
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_RED;
				#endif
				// Increment estop_timeout_counter
				estop_timeout_counter++;

				// If the commanded state is reset and the reset timeout has passed
				if ((*commanded_state == medulla_state_reset) && (estop_timeout_counter > ESTOP_TIMEOUT_LENGTH)) {
					*current_state = medulla_state_reset;
					#if defined DEUBG_LOW || defined DEBUG_HIGH
					printf("[State Machine] Entering state: Reset\n");
					#endif
				}
			}
		}
		
		// If there wasn't a falling DC clock signal, then all we need to do is rest the watchdog timer.
		WATCHDOG_TIMER_RESET;
		wait_loop();
	}

	return 1;
}

void main_estop() {
	estop(); // Run the medulla specific estop function
	estop_assert_port(&estop_port); // Then we assert the estop line
}

void amplifier_debug() {
	uint8_t computer_port_tx[64];
	uint8_t computer_port_rx[64];
	uint8_t amplifier_port_tx[64];
	uint8_t amplifier_port_rx[64];
	uint8_t data_buffer[64] = {0xA5, 0x3F, 0x02, 0x07, 0x00, 0x01, 0xB3, 0xE7, 0x0F, 0x00, 0x10, 0x3E,
	                           0xA5, 0x3E, 0x06, 0x07, 0x00, 0x01, 0xD3, 0x47, 0x0F, 0x00, 0x10, 0x3E,
	                           0xA5, 0x3F, 0x22, 0x05, 0x03, 0x02, 0x8F, 0xF9, 0x04, 0x00, 0x00, 0x00, 0xCA, 0xF1,
	                           0xA5, 0x3E, 0x26, 0x05, 0x03, 0x02, 0xEF, 0x59, 0x04, 0x00, 0x00, 0x00, 0xCA, 0xF1};
	uint8_t data_size;

	uart_port_t computer_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, computer_port_tx, 64, computer_port_rx, 64);
	uart_connect_port(&computer_port,false);

	uart_port_t amplifier_port  = uart_init_port(&PORTD, &USARTD0, uart_baud_921600, amplifier_port_tx, 64, amplifier_port_rx, 64);
	uart_connect_port(&amplifier_port,false);

	uart_tx_data(&amplifier_port,data_buffer,24);
	_delay_ms(10);
	
	uart_tx_data(&amplifier_port,data_buffer+24,28);
	_delay_ms(10);
	amplifier_port = uart_init_port(&PORTD, &USARTD0, uart_baud_115200, amplifier_port_tx, 64, amplifier_port_rx, 64);

	// Purge the amplifier input buffer before we enter this loop, so the computer doesn't get random stuff
	uart_rx_data(&amplifier_port, data_buffer, uart_received_bytes(&amplifier_port));

	while (1) {
		// Get the data from the computer and send to amplifier
		data_size = uart_rx_data(&computer_port,data_buffer,64);
		uart_tx_data(&amplifier_port,data_buffer,data_size);

		// Get data from amplifier and send it to the computer
		data_size = uart_rx_data(&amplifier_port,data_buffer,64);
		uart_tx_data(&computer_port,data_buffer,data_size);

	}

}

