#include "medulla_leg.h"

//--- Define ethercat PDO entries ---//

// RxPDO entries
medulla_state_t *leg_command_state_pdo;
uint16_t *leg_counter_pdo;
int32_t *leg_motor_current_pdo;

// TxPDO entries
uint8_t *leg_medulla_id_pdo;
medulla_state_t *leg_current_state_pdo;
uint8_t *leg_medulla_counter_pdo;
uint8_t *leg_error_flags_pdo;
uint8_t *leg_limit_switch_pdo;
uint16_t *toe_sensor_pdo;

uint32_t *motor_encoder_pdo;
uint16_t *motor_encoder_timestamp_pdo;

uint32_t *leg_encoder_pdo;
uint16_t *leg_encoder_timestamp_pdo;

uint16_t *incremental_encoder_pdo;
uint16_t *incremental_encoder_timestamp_pdo;

uint16_t *motor_voltage_pdo;
uint16_t *logic_voltage_pdo;

uint16_t *thermistor_pdo; // Pointer to all the thermistors, you can access them as an array

int16_t *measured_current_amp1_pdo;
int16_t *measured_current_amp2_pdo;

uint16_t *knee_force1_pdo;
uint16_t *knee_force2_pdo;

ecat_pdo_entry_t leg_rx_pdos[] = {{((void**)(&leg_command_state_pdo)),1},
                              {((void**)(&leg_counter_pdo)),2},
                              {((void**)(&leg_motor_current_pdo)),4}};

ecat_pdo_entry_t leg_tx_pdos[] = {{((void**)(&leg_medulla_id_pdo)),1},
                              {((void**)(&leg_current_state_pdo)),1},
                              {((void**)(&leg_medulla_counter_pdo)),1},
                              {((void**)(&leg_error_flags_pdo)),1},
                              {((void**)(&leg_limit_switch_pdo)),1},
                              {((void**)(&toe_sensor_pdo)),2},
                              {((void**)(&motor_encoder_pdo)),4},
                              {((void**)(&motor_encoder_timestamp_pdo)),2},
                              {((void**)(&leg_encoder_pdo)),4},
                              {((void**)(&leg_encoder_timestamp_pdo)),2},
                              {((void**)(&incremental_encoder_pdo)),2},
                              {((void**)(&incremental_encoder_timestamp_pdo)),2},
                              {((void**)(&motor_voltage_pdo)),2},
                              {((void**)(&logic_voltage_pdo)),2},
                              {((void**)(&thermistor_pdo)),12},
                              {((void**)(&measured_current_amp1_pdo)),2},
                              {((void**)(&measured_current_amp2_pdo)),2},
                              {((void**)(&knee_force1_pdo)),2},
                              {((void**)(&knee_force2_pdo)),2}};


// Structs for the medulla library
limit_sw_port_t limit_sw_port;
adc_port_t adc_port_a, adc_port_b;
biss_encoder_t leg_encoder, motor_encoder;
quadrature_encoder_t inc_encoder;
usart_adc_t knee_adc;
uint8_t leg_damping_cnt;
int32_t last_incremental;

// variables for filtering thermistor and voltage values
uint8_t limit_switch_counter;
uint8_t thermistor_counter;
uint16_t motor_voltage_counter;
uint16_t logic_voltage_counter;
uint8_t motor_encoder_error_counter;
uint8_t leg_encoder_error_counter;
bool leg_send_current_read;
TC0_t *leg_timestamp_timer;
int32_t prev_motor_position;
uint16_t leg_knee_adc_aux4;
uint16_t leg_therm_prev_val[6];

void leg_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog) {

	thermistor_counter = 0;
	motor_voltage_counter = 0;
	logic_voltage_counter = 0;
	leg_timestamp_timer = timestamp_timer;
	*leg_error_flags_pdo = 0;
	leg_damping_cnt = 0;


	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla Leg] Initilizing leg with ID: %04x\n",id);
	#endif
	
	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing sync managers\n");
	#endif
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_LEG_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_LEG_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing PDO entries\n");
	#endif
	ecat_configure_pdo_entries(ecat_slave, leg_rx_pdos, MEDULLA_LEG_RX_PDO_COUNT, leg_tx_pdos, MEDULLA_LEG_TX_PDO_COUNT-5); 

	#ifdef DEUBG_HIGH
	printf("[Medulla Leg] Initilizing limit switches\n");
	#endif
	switch (id) {
		case MEDULLA_LEFT_LEG_A_ID: limit_sw_port = limit_sw_init_port(&PORTK,MEDULLA_LLEG_ASIDE_LSW_MASK,&TCF0,leg_estop); break;
		case MEDULLA_LEFT_LEG_B_ID: limit_sw_port = limit_sw_init_port(&PORTK,MEDULLA_LLEG_BSIDE_LSW_MASK,&TCF0,leg_estop); break;
		case MEDULLA_RIGHT_LEG_A_ID: limit_sw_port = limit_sw_init_port(&PORTK,MEDULLA_RLEG_ASIDE_LSW_MASK,&TCF0,leg_estop); break;
		case MEDULLA_RIGHT_LEG_B_ID: limit_sw_port = limit_sw_init_port(&PORTK,MEDULLA_RLEG_BSIDE_LSW_MASK,&TCF0,leg_estop); break;
	}
	limit_switch_counter = 0;

	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing ADC ports\n");
	#endif
	adc_port_a = adc_init_port(&ADCA);
	adc_port_b = adc_init_port(&ADCB);

	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing Thermistor ADC pins\n");
	#endif
	adc_init_pin(&adc_port_a,1,thermistor_pdo+0);
	adc_init_pin(&adc_port_a,2,thermistor_pdo+1);
	adc_init_pin(&adc_port_a,3,thermistor_pdo+2);
	adc_init_pin(&adc_port_a,4,thermistor_pdo+3);
	adc_init_pin(&adc_port_a,5,thermistor_pdo+4);
	adc_init_pin(&adc_port_a,6,thermistor_pdo+5);
	
	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing voltage monitoring pins\n");
	#endif
	adc_init_pin(&adc_port_b,6,logic_voltage_pdo);
	adc_init_pin(&adc_port_b,7,motor_voltage_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing strain guage and toe sensor ADC\n");
	#endif
	knee_adc = usart_adc_init(&PORTF,&USARTF0,io_init_pin(&PORTD,4), knee_force1_pdo, knee_force2_pdo, toe_sensor_pdo, 0);

	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing motor encoder\n");
	#endif
	motor_encoder = biss_encoder_init(&PORTC,&SPIC,timestamp_timer,32,motor_encoder_pdo,motor_encoder_timestamp_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing leg encoder\n");
	#endif
	leg_encoder = biss_encoder_init(&PORTD,&SPID,timestamp_timer,32,leg_encoder_pdo,leg_encoder_timestamp_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing incremental encoder\n");
	#endif
	inc_encoder = quadrature_encoder_init(io_init_pin(&PORTD,0),io_init_pin(&PORTD,5),false,&TCF1,16384);

	#ifdef DEBUG_HIGH
	printf("[Medulla Leg] Initilizing amplifiers\n");
	#endif
	initilize_amp(true, measured_current_amp1_pdo, measured_current_amp2_pdo);


	// Start reading the ADCs
	adc_start_read(&adc_port_a);
	adc_start_read(&adc_port_b);

	while (!adc_read_complete(&adc_port_a));
	while (!adc_read_complete(&adc_port_b));

	leg_therm_prev_val[0] = thermistor_pdo[0];
	leg_therm_prev_val[1] = thermistor_pdo[1];
	leg_therm_prev_val[2] = thermistor_pdo[2];
	leg_therm_prev_val[3] = thermistor_pdo[3];
	leg_therm_prev_val[4] = thermistor_pdo[4];
	leg_therm_prev_val[5] = thermistor_pdo[5];

	*master_watchdog = leg_counter_pdo;
	*packet_counter = leg_medulla_counter_pdo;
	*leg_medulla_id_pdo = id;
	*commanded_state = leg_command_state_pdo;
	*current_state = leg_current_state_pdo;
}

inline void leg_enable_outputs(void) {
	enable_amp(true);
	limit_sw_enable_port(&limit_sw_port);
}

inline void leg_disable_outputs(void) {
	disable_amp(true);
	limit_sw_disable_port(&limit_sw_port);
}

void leg_update_inputs(uint8_t id) {
	// Since reading from the knee ADC takes so long, we start it first.
	//usart_adc_start_read(&knee_adc);

	// Start reading the ADCs
	adc_start_read(&adc_port_a);
	adc_start_read(&adc_port_b);
	
	// Start reading from the encoders
	biss_encoder_start_reading(&motor_encoder);
	biss_encoder_start_reading(&leg_encoder);

	// while we are waiting for things to complete, get the limit switch state
	*leg_limit_switch_pdo = limit_sw_get_port(&limit_sw_port);


	// now wait for things to complete
	while (!adc_read_complete(&adc_port_a));
	while (!adc_read_complete(&adc_port_b));
 	while (!biss_encoder_read_complete(&motor_encoder));
	while (!biss_encoder_read_complete(&leg_encoder));
	//while (!usart_adc_read_complete(&knee_adc));

	cli();
	last_incremental = *incremental_encoder_pdo;
	*incremental_encoder_pdo = quadrature_encoder_get_value(&inc_encoder);
	*incremental_encoder_timestamp_pdo = leg_timestamp_timer->CNT;
	sei();

	// make sure our encoder data is accurate, if it is, then update, if it's not, then increment the error coutner.
	prev_motor_position = (int32_t)*motor_encoder_pdo;
	if (biss_encoder_data_valid(&motor_encoder)) {
		biss_encoder_process_data(&motor_encoder);
	}
	else {
		*leg_error_flags_pdo |= medulla_error_encoder;
		motor_encoder_error_counter++;
	}
	
	if (biss_encoder_data_valid(&leg_encoder)) {
		biss_encoder_process_data(&leg_encoder);
	}
	else {
		*leg_error_flags_pdo |= medulla_error_encoder;
		leg_encoder_error_counter++;
	}

	//usart_adc_process_data(&knee_adc);

	

	if (((leg_therm_prev_val[0]>thermistor_pdo[0]) && (leg_therm_prev_val[0]-thermistor_pdo[0] < 50)) ||
	    ((leg_therm_prev_val[0]<thermistor_pdo[0]) && (thermistor_pdo[0]-leg_therm_prev_val[0] < 50)))
		leg_therm_prev_val[0] = thermistor_pdo[0];

	if (((leg_therm_prev_val[1]>thermistor_pdo[1]) && (leg_therm_prev_val[1]-thermistor_pdo[1] < 50)) ||
	    ((leg_therm_prev_val[1]<thermistor_pdo[1]) && (thermistor_pdo[1]-leg_therm_prev_val[1] < 50)))
		leg_therm_prev_val[1] = thermistor_pdo[1];

	if (((leg_therm_prev_val[2]>thermistor_pdo[2]) && (leg_therm_prev_val[2]-thermistor_pdo[2] < 50)) ||
	    ((leg_therm_prev_val[2]<thermistor_pdo[2]) && (thermistor_pdo[2]-leg_therm_prev_val[2] < 50)))
		leg_therm_prev_val[2] = thermistor_pdo[2];

	if (((leg_therm_prev_val[3]>thermistor_pdo[3]) && (leg_therm_prev_val[3]-thermistor_pdo[3] < 1000)) ||
	    ((leg_therm_prev_val[3]<thermistor_pdo[3]) && (thermistor_pdo[3]-leg_therm_prev_val[3] < 1000)))
		leg_therm_prev_val[3] = thermistor_pdo[3];

	if (((leg_therm_prev_val[4]>thermistor_pdo[4]) && (leg_therm_prev_val[4]-thermistor_pdo[4] < 50)) ||
	    ((leg_therm_prev_val[4]<thermistor_pdo[4]) && (thermistor_pdo[4]-leg_therm_prev_val[4] < 50)))
		leg_therm_prev_val[4] = thermistor_pdo[4];

	if (((leg_therm_prev_val[5]>thermistor_pdo[5]) && (leg_therm_prev_val[5]-thermistor_pdo[5] < 50)) ||
	    ((leg_therm_prev_val[5]<thermistor_pdo[5]) && (thermistor_pdo[5]-leg_therm_prev_val[5] < 50)))
		leg_therm_prev_val[5] = thermistor_pdo[5];

	//leg_send_current_read = true;
}

bool leg_run_halt(uint8_t id) {
	leg_damping_cnt += 1;
	static int32_t diff = 0;
	diff = (int32_t)(*incremental_encoder_pdo)-last_incremental;
	diff = MOD(diff + (((int32_t)1)<<15), (((int32_t)1)<<16)) - (((int32_t)1)<<15);
	if ((diff <= 10) && (diff >= -10)) {
		set_amp_output(0);
		if (leg_damping_cnt > 100)
			return false;
		else
			return true;
	}

	diff = diff * DAMPING_GAIN_CONSTANT * DAMPING_GAIN;
	if (diff > DAMPING_CURRENT_LIMIT)
		diff = DAMPING_CURRENT_LIMIT;
	else if (diff < (-1*DAMPING_CURRENT_LIMIT))
		diff = -1*DAMPING_CURRENT_LIMIT;

	set_amp_output(diff);

	return true;
}

inline void leg_update_outputs(uint8_t id) {
	set_amp_output(*leg_motor_current_pdo);
}

inline void leg_estop(void) {
	disable_pwm();
}

void leg_wait_loop() {
	//if (leg_send_current_read == true) {
	//	send_current_read(true);
	//	leg_send_current_read = false;
	//}
	//else
	//	check_current_read(true);
}

bool leg_check_error(uint8_t id) {
	#ifdef ERROR_CHECK_LIMIT_SWITCH
	if (limit_sw_get_port(&limit_sw_port)) {
		limit_switch_counter ++;
	}
	else if (limit_switch_counter > 0)
		limit_switch_counter --;
	
	if (limit_switch_counter > 50) {
		#if defined DEBUG_LOW || DEBUG_HIGH
		printf("[Medulla Leg] Limit switch error: %d\n",limit_sw_get_port(&limit_sw_port));
		#endif
		*leg_error_flags_pdo |= medulla_error_limit_switch;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_THERMISTORS
	// Do filtering on thermistor values 
	if ((leg_therm_prev_val[0] < THERMISTOR_MAX_VAL) ||
        (leg_therm_prev_val[1] < THERMISTOR_MAX_VAL) ||
        (leg_therm_prev_val[2] < THERMISTOR_MAX_VAL) ||
        (leg_therm_prev_val[3] < THERMISTOR_MAX_VAL) ||
        (leg_therm_prev_val[4] < THERMISTOR_MAX_VAL) ||
        (leg_therm_prev_val[5] < THERMISTOR_MAX_VAL)) {
		thermistor_counter++;
	}
	else if (thermistor_counter > 0)
		thermistor_counter--;
	if (thermistor_counter > 100) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla Leg] Thermistor error.\n");
		#endif
		*leg_error_flags_pdo |= medulla_error_thermistor;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_MOTOR_VOLTAGE
	// Do filter on motor voltage
	if ((*motor_voltage_pdo < MOTOR_VOLTAGE_DANGER_MAX) && (*motor_voltage_pdo > MOTOR_VOLTAGE_DANGER_MIN))
		motor_voltage_counter++;
	else if (motor_voltage_counter > 0)
		motor_voltage_counter--;

	// Check if we are in the motor voltage danger range
	if (motor_voltage_counter > 500) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla Leg] Motor voltage error.");
		#endif
		*leg_error_flags_pdo |= medulla_error_motor_voltage;
		return true;
	}
	#endif

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
		*leg_error_flags_pdo |= medulla_error_logic_voltage;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_ENCODER
	// Check the encoder error counters
	if ((motor_encoder_error_counter > 10) || (leg_encoder_error_counter > 10)) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla Leg] Encoder read error\n");
		#endif
		*leg_error_flags_pdo |= medulla_error_encoder;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_AMP
	// Check that there are no errors on the amplifier
	if ((AMP_DIO_PORT.IN & AMP1_ERROR_bm) || (AMP_DIO_PORT.IN & AMP2_ERROR_bm)) {
		#if defined DEBUG_LOW || DEBUG_HIGH
		printf("[Medulla Leg] Amplifier reported error\n");
		#endif
		*leg_error_flags_pdo |= medulla_error_amplifier;
	}
	#endif

	// If none of the above caused us to return true, then there are no errors and we return false
	return false;

}

bool leg_check_halt(uint8_t id) {
	#ifndef MEDULLA_USE_HALT
	return false;
	#else

	static uint32_t maxCounts = 0;
	static uint32_t minCounts = 0;
	static int32_t damping_location = 0;
	static int32_t diff = 0;
	diff = (int32_t)(*incremental_encoder_pdo)-last_incremental;
	diff = MOD(diff + (((int32_t)1)<<15), (((int32_t)1)<<16)) - (((int32_t)1)<<15);


	int8_t countDirection = 1;
	// First check if the encoder value is even reasonable
	if ((((int32_t)(*motor_encoder_pdo) - prev_motor_position) > MAX_ACCEPTABLE_ENCODER_CHANGE) ||
	    (((int32_t)(*motor_encoder_pdo) - prev_motor_position) < (MAX_ACCEPTABLE_ENCODER_CHANGE*-1))) {
		// We have a bad encoder value, just ignore it and go on.
		return false;
	}

	switch (id) {
		case MEDULLA_LEFT_LEG_A_ID:
			maxCounts = LOC_TO_COUNTS(LEG_A_MOTOR_MAX_LOC-LEG_LOC_SAFETY_DISTANCE,LEG_A_CALIB_LOC,LEFT_TRAN_A_CALIB_VAL,LEFT_TRAN_A_RAD_PER_CNT);
			minCounts = LOC_TO_COUNTS(LEG_A_MOTOR_MIN_LOC+LEG_LOC_SAFETY_DISTANCE,LEG_A_CALIB_LOC,LEFT_TRAN_A_CALIB_VAL,LEFT_TRAN_A_RAD_PER_CNT);
			countDirection = (LEFT_TRAN_A_RAD_PER_CNT > 0) ? 1 : -1;
			damping_location = diff*((diff > 0) ? diff : -1*diff)*(int32_t)(-1.0*LEFT_MOTOR_A_DIRECTION/(DAMPING_MAX_CURRENT*ACCEL_PER_AMP*LEFT_TRAN_A_RAD_PER_CNT*24824));
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			maxCounts = LOC_TO_COUNTS(LEG_B_MOTOR_MAX_LOC-LEG_LOC_SAFETY_DISTANCE,LEG_B_CALIB_LOC,LEFT_TRAN_B_CALIB_VAL,LEFT_TRAN_B_RAD_PER_CNT);
			minCounts = LOC_TO_COUNTS(LEG_B_MOTOR_MIN_LOC+LEG_LOC_SAFETY_DISTANCE,LEG_B_CALIB_LOC,LEFT_TRAN_B_CALIB_VAL,LEFT_TRAN_B_RAD_PER_CNT);
			countDirection = (LEFT_TRAN_B_RAD_PER_CNT > 0) ? 1 : -1;
			damping_location = diff*((diff > 0) ? diff : -1*diff)*(int32_t)(-1.0*LEFT_MOTOR_B_DIRECTION/(DAMPING_MAX_CURRENT*ACCEL_PER_AMP*LEFT_TRAN_B_RAD_PER_CNT*24824));
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			maxCounts = LOC_TO_COUNTS(LEG_A_MOTOR_MAX_LOC-LEG_LOC_SAFETY_DISTANCE,LEG_A_CALIB_LOC,RIGHT_TRAN_A_CALIB_VAL,RIGHT_TRAN_A_RAD_PER_CNT);
			minCounts = LOC_TO_COUNTS(LEG_A_MOTOR_MIN_LOC+LEG_LOC_SAFETY_DISTANCE,LEG_A_CALIB_LOC,RIGHT_TRAN_A_CALIB_VAL,RIGHT_TRAN_A_RAD_PER_CNT);
			countDirection = (RIGHT_TRAN_A_RAD_PER_CNT > 0) ? 1 : -1;
			damping_location = diff*((diff > 0) ? diff : -1*diff)*(int32_t)(-1.0*RIGHT_MOTOR_A_DIRECTION/(DAMPING_MAX_CURRENT*ACCEL_PER_AMP*RIGHT_TRAN_A_RAD_PER_CNT*24824));
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			maxCounts = LOC_TO_COUNTS(LEG_B_MOTOR_MAX_LOC-LEG_LOC_SAFETY_DISTANCE,LEG_B_CALIB_LOC,RIGHT_TRAN_B_CALIB_VAL,RIGHT_TRAN_B_RAD_PER_CNT);
			minCounts = LOC_TO_COUNTS(LEG_B_MOTOR_MIN_LOC+LEG_LOC_SAFETY_DISTANCE,LEG_B_CALIB_LOC,RIGHT_TRAN_B_CALIB_VAL,RIGHT_TRAN_B_RAD_PER_CNT);
			countDirection = (RIGHT_TRAN_B_RAD_PER_CNT > 0) ? 1 : -1;
			damping_location = diff*((diff > 0) ? diff : -1*diff)*(int32_t)(-1.0*RIGHT_MOTOR_B_DIRECTION/(DAMPING_MAX_CURRENT*ACCEL_PER_AMP*RIGHT_TRAN_B_RAD_PER_CNT*24824));
			break;
	}

	damping_location = damping_location + ((int32_t)(*motor_encoder_pdo));

	if ((countDirection > 0) && 
	    ((damping_location > maxCounts) || (damping_location < minCounts))) {
		return true;
	}
	
	if ((countDirection < 0) && 
	    ((damping_location < maxCounts) || (damping_location > minCounts))) {
		return true;
	}


	return false;
	#endif
}

void leg_reset_error() {
	*leg_error_flags_pdo = 0;
	thermistor_counter = 0;
	motor_voltage_counter = 0;
	logic_voltage_counter = 0;
	motor_encoder_error_counter = 0;
	leg_encoder_error_counter = 0;
	leg_damping_cnt = 0;
}

