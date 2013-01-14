#include "medulla_boom.h"

//--- Define ethercat PDO entries ---//

// RxPDO entries
medulla_state_t *boom_command_state_pdo;
uint16_t *boom_counter_pdo;

// TxPDO entries
uint8_t *boom_medulla_id_pdo;
medulla_state_t *boom_current_state_pdo;
uint8_t *boom_medulla_counter_pdo;
uint8_t *boom_error_flags_pdo;

uint32_t *x_encoder_pdo;
uint16_t *x_encoder_timestamp_pdo;

uint32_t *pitch_encoder_pdo;
uint16_t *pitch_encoder_timestamp_pdo;

uint32_t *z_encoder_pdo;
uint16_t *z_encoder_timestamp_pdo;

uint16_t *logic_voltage_pdo;

ecat_pdo_entry_t boom_rx_pdos[] = {{((void**)(&boom_command_state_pdo)),1},
                                   {((void**)(&boom_counter_pdo)),2}};

ecat_pdo_entry_t boom_tx_pdos[] = {{((void**)(&boom_medulla_id_pdo)),1},
                              {((void**)(&boom_current_state_pdo)),1},
                              {((void**)(&boom_medulla_counter_pdo)),1},
                              {((void**)(&boom_error_flags_pdo)),1},
                              {((void**)(&x_encoder_pdo)),4},
                              {((void**)(&x_encoder_timestamp_pdo)),2},
                              {((void**)(&pitch_encoder_pdo)),4},
                              {((void**)(&pitch_encoder_timestamp_pdo)),2},
                              {((void**)(&z_encoder_pdo)),4},
                              {((void**)(&z_encoder_timestamp_pdo)),2},
                              {((void**)(&logic_voltage_pdo)),2}};


// Structs for the medulla library
adc_port_t adc_port_b;
hengstler_ssi_encoder_t x_encoder, pitch_encoder, z_encoder;
io_pin_t sync_pin;

// variables for filtering thermistor and voltage values
uint16_t logic_voltage_counter;
uint8_t boom_damping_cnt;

void boom_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog) {

	logic_voltage_counter = 0;
	boom_damping_cnt = 0;

	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla Boom] Initilizing boom with ID: %04x\n",id);
	#endif
	
	#ifdef DEBUG_HIGH
	printf("[Medulla Boom] Initilizing sync managers\n");
	#endif
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_BOOM_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_BOOM_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla Boom] Initilizing PDO entries\n");
	#endif
	ecat_configure_pdo_entries(ecat_slave, boom_rx_pdos, MEDULLA_BOOM_RX_PDO_COUNT, boom_tx_pdos, MEDULLA_BOOM_TX_PDO_COUNT); 

	#ifdef DEBUG_HIGH
	printf("[Medulla Boom] Initilizing ADC port\n");
	#endif
	adc_port_b = adc_init_port(&ADCB);

	#ifdef DEBUG_HIGH
	printf("[Medulla Boom] Initilizing voltage monitoring pins\n");
	#endif
	adc_init_pin(&adc_port_b,6,logic_voltage_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Boom] Initilizing X encoder\n");
	#endif
	x_encoder = hengstler_ssi_encoder_init(&PORTC,&SPIC,timestamp_timer,x_encoder_pdo,x_encoder_timestamp_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Boom] Initilizing pitch encoder\n");
	#endif
	pitch_encoder = hengstler_ssi_encoder_init(&PORTD,&SPID,timestamp_timer,pitch_encoder_pdo,pitch_encoder_timestamp_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Boom] Initilizing Z encoder\n");
	#endif
	z_encoder = hengstler_ssi_encoder_init(&PORTF,&SPIF,timestamp_timer,z_encoder_pdo,z_encoder_timestamp_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Boom] Initilizing force plate sync output");
	#endif
	sync_pin = io_init_pin(&PORTB,0);
	io_set_direction(sync_pin, io_input);
	io_set_output(sync_pin, io_low);
	
	*master_watchdog = boom_counter_pdo;
	*packet_counter = boom_medulla_counter_pdo;
	*boom_medulla_id_pdo = id;
	*commanded_state = boom_command_state_pdo;
	*current_state = boom_current_state_pdo;
}

inline void boom_enable_outputs(void) {
	// Pull sync pin low
	io_set_direction(sync_pin,io_output);
}

inline void boom_disable_outputs(void) {
	// Pull sync pin high
	io_set_direction(sync_pin,io_input);
}

void boom_update_inputs(uint8_t id) {
	// Start reading the ADCs
	adc_start_read(&adc_port_b);

	// Start reading from the encoders
	hengstler_ssi_encoder_start_reading(&x_encoder);
	hengstler_ssi_encoder_start_reading(&pitch_encoder);
	hengstler_ssi_encoder_start_reading(&z_encoder);

	// now wait for things to complete
	while (!adc_read_complete(&adc_port_b));
	while (!hengstler_ssi_encoder_read_complete(&x_encoder));
	while (!hengstler_ssi_encoder_read_complete(&pitch_encoder));
	while (!hengstler_ssi_encoder_read_complete(&z_encoder));

	// make sure our encoder data is accurate, if it is, then update, if it's not, then increment the error coutner.
	hengstler_ssi_encoder_process_data(&x_encoder);
	hengstler_ssi_encoder_process_data(&pitch_encoder);
	hengstler_ssi_encoder_process_data(&z_encoder);

//	printf("%lu\n",*x_encoder_pdo);
}

bool boom_run_halt(uint8_t id) {
	boom_damping_cnt += 1;
	if (boom_damping_cnt > 100)
		return false;
	return true;
}

inline void boom_update_outputs(uint8_t id) {
}

inline void boom_estop(void) {
}

void boom_wait_loop(void) {
}

bool boom_check_error(uint8_t id) {
	#ifdef ERROR_CHECK_LOGIC_VOLTAGE
	// Do filter on logic voltage
	if (*logic_voltage_pdo < LOGIC_VOLTAGE_MIN)
		logic_voltage_counter++;
	else if (logic_voltage_counter > 0)
		logic_voltage_counter--;

	// Check if we are in the logic voltage danger range
	if (logic_voltage_counter > 500) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla Leg] Logic voltage error.\n");
		#endif
		*boom_error_flags_pdo |= medulla_error_logic_voltage;
		return true;
	}
	#endif

	// If none of the above caused us to return true, then there are no errors and we return false
	return false;

}

bool boom_check_halt(uint8_t id) {
	return false;
}

void boom_reset_error() {
	*boom_error_flags_pdo = 0;
	logic_voltage_counter = 0;
	boom_damping_cnt = 0;
}

