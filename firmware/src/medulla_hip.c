#include "medulla_hip.h" 
//--- Define ethercat PDO entries ---//

// RxPDO entries
medulla_state_t *hip_command_state_pdo;
uint16_t *hip_counter_pdo;
int32_t *hip_motor_current_pdo;

// TxPDO entries
uint8_t *hip_medulla_id_pdo;
medulla_state_t *hip_current_state_pdo;
uint8_t *hip_medulla_counter_pdo;
uint8_t *hip_error_flags_pdo;
uint8_t *hip_limit_switch_pdo;

uint32_t *hip_encoder_pdo;
uint16_t *hip_encoder_timestamp_pdo;

uint16_t *hip_motor_voltage_pdo;
uint16_t *hip_logic_voltage_pdo;

uint16_t *hip_thermistor_pdo; // Pointer to all the thermistors, you can access them as an array

int16_t *hip_measured_current_pdo;

//pointer to desired IMU data pdo
uint8_t *hip_imu_data_pdo;

uint16_t *hip_incremental_encoder_pdo;
uint16_t *hip_incremental_encoder_timestamp_pdo;

ecat_pdo_entry_t hip_rx_pdos[] = {{((void**)(&hip_command_state_pdo)),1},
                              {((void**)(&hip_counter_pdo)),2},
                              {((void**)(&hip_motor_current_pdo)),4}};

ecat_pdo_entry_t hip_tx_pdos[] = {{((void**)(&hip_medulla_id_pdo)),1},
                              {((void**)(&hip_current_state_pdo)),1},
                              {((void**)(&hip_medulla_counter_pdo)),1},
                              {((void**)(&hip_error_flags_pdo)),1},
                              {((void**)(&hip_limit_switch_pdo)),1},
                              {((void**)(&hip_encoder_pdo)),4},
                              {((void**)(&hip_encoder_timestamp_pdo)),2},
                              {((void**)(&hip_motor_voltage_pdo)),2},
                              {((void**)(&hip_logic_voltage_pdo)),2},
                              {((void**)(&hip_thermistor_pdo)),6},
                              {((void**)(&hip_measured_current_pdo)),2},
//							  {((void**)(&hip_imu_data_pdo)),64},
                              {((void**)(&hip_incremental_encoder_pdo)),2},
                              {((void**)(&hip_incremental_encoder_timestamp_pdo)),2}};


// Structs for the medulla library
limit_sw_port_t hip_limit_sw_port;
adc_port_t adc_port_a, adc_port_b;
renishaw_ssi_encoder_t hip_encoder;
quadrature_encoder_t hip_inc_encoder;
#ifdef USING_IMU
IMU_data_t hip_imu;

//IMU read pacing Flag
uint8_t hip_imu_pace_flag;
#endif

// variables for filtering thermistor and voltage values
uint8_t hip_thermistor_counter;
uint16_t hip_motor_voltage_counter;
uint16_t hip_logic_voltage_counter;
bool hip_send_current_read;
TC0_t *hip_timestamp_timer;

void hip_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter,TC0_t *timestamp_timer, uint16_t **master_watchdog) {

	hip_thermistor_counter = 0;
	hip_motor_voltage_counter = 0;
	hip_logic_voltage_counter = 0;
	hip_timestamp_timer = timestamp_timer;

	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla Hip] Initilizing leg with ID: %04x\n",id);
	#endif
	
	#ifdef DEBUG_HIGH
	printf("[Medulla Hip] Initilizing sync managers\n");
	#endif
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_HIP_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_HIP_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla Hip] Initilizing PDO entries\n");
	#endif
	ecat_configure_pdo_entries(ecat_slave, hip_rx_pdos, MEDULLA_HIP_RX_PDO_COUNT, hip_tx_pdos, 13); 

	#ifdef DEUBG_HIGH
	printf("[Medulla Hip] Initilizing limit switches\n");
	#endif
	hip_limit_sw_port = limit_sw_init_port(&PORTK,1,&TCF0,hip_estop);

	#ifdef DEBUG_HIGH
	printf("[Medulla Hip] Initilizing ADC ports\n");
	#endif
	adc_port_a = adc_init_port(&ADCA);
	adc_port_b = adc_init_port(&ADCB);

	#ifdef DEBUG_HIGH
	printf("[Medulla Hip] Initilizing Thermistor ADC pins\n");
	#endif
	adc_init_pin(&adc_port_a,1,hip_thermistor_pdo+0);
	adc_init_pin(&adc_port_a,2,hip_thermistor_pdo+1);
	adc_init_pin(&adc_port_a,3,hip_thermistor_pdo+2);

	#ifdef DEBUG_HIGH
	printf("[Medulla Hip] Initilizing voltage monitoring pins\n");
	#endif
	adc_init_pin(&adc_port_b,6,hip_logic_voltage_pdo);
	adc_init_pin(&adc_port_b,7,hip_motor_voltage_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Hip] Initilizing hip encoder\n");
	#endif
	hip_encoder = renishaw_ssi_encoder_init(&PORTC,&SPIC,timestamp_timer,hip_encoder_pdo,hip_encoder_timestamp_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Hip] Initilizing incremental encoder\n");
	#endif
	hip_inc_encoder = quadrature_encoder_init(io_init_pin(&PORTD,0),io_init_pin(&PORTD,5),false,&TCF1,16384);

	#ifdef DEBUG_HIGH
	printf("[Medulla Hip] Initilizing amplifiers\n");
	#endif
	initilize_amp(false, hip_measured_current_pdo, 0);

	#ifdef USING_IMU
	if (id == MEDULLA_RIGHT_HIP_ID) {
		#ifdef DEBUG_HIGH
		printf("[Medulla Hip] Initializin IMU\n");
		#endif
		//IMU at RS232 interface (USARTE0)
		init_IMU(&PORTE, &USARTE0, &hip_imu, hip_imu_data_pdo);
		hip_imu_pace_flag = 0;
	}
	#endif
	
	*master_watchdog = hip_counter_pdo;
	*packet_counter = hip_medulla_counter_pdo;
	*hip_medulla_id_pdo = id;
	*commanded_state = hip_command_state_pdo;
	*current_state = hip_current_state_pdo;
}

inline void hip_enable_outputs(void) {
	enable_amp(false);
}

inline void hip_disable_outputs(void) {
	disable_amp(false);
}

void hip_update_inputs(uint8_t id) {
	// Start reading the ADCs
	adc_start_read(&adc_port_a);
	adc_start_read(&adc_port_b);

	// Start reading from the encoders
	renishaw_ssi_encoder_start_reading(&hip_encoder);

	// while we are waiting for things to complete, get the limit switch state
	*hip_limit_switch_pdo = limit_sw_get_port(&hip_limit_sw_port);

	// now wait for things to complete
	while (!adc_read_complete(&adc_port_a));
	while (!adc_read_complete(&adc_port_b));
	while (!renishaw_ssi_encoder_read_complete(&hip_encoder));

	cli();
	*hip_incremental_encoder_pdo = quadrature_encoder_get_value(&hip_inc_encoder);
	*hip_incremental_encoder_timestamp_pdo = hip_timestamp_timer->CNT;
	sei();

	renishaw_ssi_encoder_process_data(&hip_encoder);
	
	hip_send_current_read = true;

	#ifdef USING_IMU
	if (id == MEDULLA_RIGHT_HIP_ID) {
		if(hip_imu_pace_flag){
			hip_imu_pace_flag = 0;
			IMU_request_orientation(&hip_imu);
		}
		else {
			while(IMU_received_orientation(&hip_imu) == false);
			IMU_process_orientation(&hip_imu);
			hip_imu_pace_flag = 1;
		}
	}
	#endif
}

bool hip_run_halt(uint8_t id) {
	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Run Halt\n");
	#endif
	return false;
}

inline void hip_update_outputs(uint8_t id) {
	set_amp_output(*hip_motor_current_pdo);
}

inline void hip_estop(void) {
	disable_pwm();
}

void hip_wait_loop() {
	if (hip_send_current_read == true) {
		send_current_read(false);
		hip_send_current_read = false;
	}
	else
		check_current_read(false);
}

bool hip_check_error(uint8_t id) {
//	return false;
	#ifdef ERROR_CHECK_LIMIT_SWITCH
	if (limit_sw_get_port(&hip_limit_sw_port)) { // If any of the limit switches are pressed, go to error
		#if defined DEBUG_LOW || DEBUG_HIGH
		printf("[Medulla Hip] Limit switch error.\n");
		#endif
		*hip_error_flags_pdo |= medulla_error_limit_switch;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_THERMISTORS
	// Do filtering on thermistor values
	if ((hip_thermistor_pdo[0] < THERMISTOR_MAX_VAL) ||
        (hip_thermistor_pdo[1] < THERMISTOR_MAX_VAL) ||
        (hip_thermistor_pdo[2] < THERMISTOR_MAX_VAL))
		hip_thermistor_counter++;
	else if (hip_thermistor_counter > 0)
		hip_thermistor_counter--;

	// Check thermistor values
	if (hip_thermistor_counter > 100) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla Hip] Thermistor error.\n");
		#endif
		*hip_error_flags_pdo |= medulla_error_thermistor;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_MOTOR_VOLTAGE
	// Do filter on motor voltage
	if ((*hip_motor_voltage_pdo < MOTOR_VOLTAGE_DANGER_MAX) && (*hip_motor_voltage_pdo > MOTOR_VOLTAGE_DANGER_MIN))
		hip_motor_voltage_counter++;
	else if (hip_motor_voltage_counter > 0)
		hip_motor_voltage_counter--;

	// Check if we are in the motor voltage danger range
	if (hip_motor_voltage_counter > 500) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla Hip] Motor voltage error.\n");
		#endif
		*hip_error_flags_pdo |= medulla_error_motor_voltage;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_LOGIC_VOLTAGE
	// Do filter on logic voltage
	if (*hip_logic_voltage_pdo < LOGIC_VOLTAGE_MIN)
		hip_logic_voltage_counter++;
	else if (hip_logic_voltage_counter > 0)
		hip_logic_voltage_counter--;

	// Check if we are in the logic voltage danger range
	if (hip_logic_voltage_counter > 500) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla Hip] Logic voltage error.\n");
		#endif
		*hip_error_flags_pdo |= medulla_error_logic_voltage;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_AMP
	// Check that there are no errors on the amplifier
	if ((AMP_DIO_PORT.IN & AMP1_ERROR_bm) || (AMP_DIO_PORT.IN & AMP2_ERROR_bm)) {
		#if defined DEBUG_LOW || DEBUG_HIGH
		printf("[Medulla Leg] Amplifier reported error\n");
		#endif
		*hip_error_flags_pdo |= medulla_error_amplifier;
	}
	#endif

	// If none of the above caused us to return true, then there are no errors and we return false
	return false;

}

bool hip_check_halt(uint8_t id) {
//	if (0) {
//		*error_flags_pdo |= halt_error;
//		return true;
//	}	
	return false;
}

void hip_reset_error() {
	*hip_error_flags_pdo = 0;
	hip_thermistor_counter = 0;
	hip_motor_voltage_counter = 0;
	hip_logic_voltage_counter = 0;
}

