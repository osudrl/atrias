#include "medulla_imu.h"

// RxPDO entries
extern medulla_state_t *imu_command_state_pdo;
extern uint16_t *imu_counter_pdo;
extern uint16_t *imu_command_pdo;

// TxPDO entries
extern uint8_t *imu_medulla_id_pdo;
extern medulla_state_t *imu_current_state_pdo;
extern uint8_t *imu_medulla_counter_pdo;
extern uint8_t *imu_error_flags_pdo;

extern uint32_t *XAngRate_pdo;
extern uint32_t *XAccel_pdo;
extern uint32_t *YAngRate_pdo;
extern uint32_t *YAccel_pdo;
extern uint32_t *ZAngRate_pdo;
extern uint32_t *ZAccel_pdo;
extern uint32_t *M11_pdo;
extern uint32_t *M12_pdo;
extern uint32_t *M13_pdo;
extern uint32_t *M21_pdo;
extern uint32_t *M22_pdo;
extern uint32_t *M23_pdo;
extern uint32_t *M31_pdo;
extern uint32_t *M32_pdo;
extern uint32_t *M33_pdo;


ecat_pdo_entry_t imu_rx_pdos[] = {{((void**)(&imu_command_state_pdo)),1},
                                  {((void**)(&imu_counter_pdo)),2},
                                  {((void**)(&imu_command_pdo)),2}};

ecat_pdo_entry_t imu_tx_pdos[] = {{((void**)(&imu_medulla_id_pdo)),1},
                                  {((void**)(&imu_current_state_pdo)),1},
                                  {((void**)(&imu_medulla_counter_pdo)),1},
                                  {((void**)(&imu_error_flags_pdo)),1},
                                  {((void**)(&XAngRate_pdo)),4},
                                  {((void**)(&XAccel_pdo)),4},
                                  {((void**)(&YAngRate_pdo)),4},
                                  {((void**)(&YAccel_pdo)),4},
                                  {((void**)(&ZAngRate_pdo)),4},
                                  {((void**)(&ZAccel_pdo)),4},
                                  {((void**)(&M11_pdo)),4},
                                  {((void**)(&M12_pdo)),4},
                                  {((void**)(&M13_pdo)),4},
                                  {((void**)(&M21_pdo)),4},
                                  {((void**)(&M22_pdo)),4},
                                  {((void**)(&M23_pdo)),4},
                                  {((void**)(&M31_pdo)),4},
                                  {((void**)(&M32_pdo)),4},
                                  {((void**)(&M33_pdo)),4}};

//--- Define the interrupt functions --- //
// Debug port
UART_USES_PORT(USARTE0)

// Ethercat on port E
ECAT_USES_PORT(SPIE);

// Interrupt for handling watchdog (we don't need a driver for this)
ISR(TCE1_OVF_vect) {
	WATCHDOG_TIMER.INTCTRLA = TC_OVFINTLVL_OFF_gc;
	LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK);
	printf("[ERROR] Watchdog timer overflow\n");
	while(1);
}

//extern ecat_pdo_entry_t imu_rx_pdos[];
//extern ecat_pdo_entry_t imu_tx_pdos[];
extern imu_sampling_settings_t current_settings;

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

	// Initilizing timestamp counter
	TIMESTAMP_COUNTER.CTRLA = TC_CLKSEL_DIV2_gc;

	// Initilize the EtherCAT
	ecat_port = ecat_init_slave(&PORTE,&SPIE,io_init_pin(&PORTE,0),io_init_pin(&PORTE,1));
	// set the the IRQ pin so it sets the IRQ flags on the falling edge so we can check that for the DC clock
	PORTE.PIN1CTRL = PORT_ISC_FALLING_gc;
	PORTE.INT0MASK = 0b10;
	// Configure sync managers and pdos
	ecat_init_sync_managers(&ecat_port, ecat_rx_sm_buffer, 5, 0x1000, ecat_tx_sm_buffer, 64, 0x2000);
	ecat_configure_pdo_entries(&ecat_port, imu_rx_pdos, 3, imu_tx_pdos, 19);

	// Wait for a second so all the hardware can initilize first
	_delay_ms(1000);

	//imu_initialize(medulla_id, &ecat_port, ecat_tx_sm_buffer, ecat_rx_sm_buffer, &commanded_state, &current_state, &packet_counter, &TIMESTAMP_COUNTER, &master_watchdog_counter);
	
	imu_init_uart();
	imu_init_pdos();
	
	// This delay is needed to allow the IMU time to initialize
	LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_RED;
	_delay_ms(20000);
	imu_soft_reset();
	_delay_ms(20000);
	
	imu_get_sampling_settings(&current_settings);
	current_settings.enable_quaternion                = false;
	current_settings.enable_magnetometer              = false;
	current_settings.enable_north_compensation        = false;
	current_settings.north_compensation_time_constant = 10;
	current_settings.enable_up_compensation           = true;
	current_settings.up_compensation_time_constant    = 70;
	current_settings.gyro_accel_filter_length         = 15;
	current_settings.magnetometer_filter_length       = 17;
	imu_set_sampling_settings(&current_settings);
	
	// Now that everything is set up, start the watchdog timer
	WATCHDOG_TIMER.INTCTRLA = TC_OVFINTLVL_HI_gc;
	WATCHDOG_TIMER.CTRLA = TC_CLKSEL_DIV4_gc;

	#ifdef ENABLE_LEDS
	LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_GREEN;
	#endif

	while (1)
	{	
		//Read new commands from the ethercat slave
		ecat_read_rx_sm(&ecat_port);

		// Run state machine, update outputs
		imu_run_state_machine();		

		//Send the new sensor data to the ethercat slave
		ecat_write_tx_sm(&ecat_port);

		// feed the watchdog timer
		WATCHDOG_TIMER_RESET;
	}
	return 1;
}


