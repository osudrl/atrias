#include "../include/medulla_imu.h"
#include "../include/medulla.h"

kvh_data_t kvh_data;

//--- Define ethercat PDO entries ---//

#define SHIFT_1BYTE(X)	(X<<8)
#define SHIFT_2BYTE(X)	(X<<16)
#define SHIFT_3BYTE(X)	(X<<24)

uint8_t imu_rx_buffer[100];
uint8_t imu_tx_buffer[100];

uint32_t buffer[1000];
uint32_t counter = 0;

uart_port_t uart_debug;

// RxPDO entries
medulla_state_t *imu_command_state_pdo;
uint16_t *imu_counter_pdo;
int32_t *imu_command_pdo;

// TxPDO entries
uint8_t *imu_medulla_id_pdo;
medulla_state_t *imu_current_state_pdo;
uint8_t *imu_medulla_counter_pdo;
uint8_t *imu_error_flags_pdo;
uint32_t *XAngDelta_pdo;
uint32_t *XAccel_pdo;
uint32_t *YAngDelta_pdo;
uint32_t *YAccel_pdo;
uint32_t *ZAngDelta_pdo;
uint32_t *ZAccel_pdo;
uint8_t  *Status_pdo;
uint8_t  *Seq_pdo;
int16_t  *Temp_pdo;

void populate_byte_to_data(const uint8_t* data_byte,uint32_t* data);

ecat_pdo_entry_t imu_rx_pdos[] = {{((void**)(&imu_command_state_pdo)),1},
							  {((void**)(&imu_counter_pdo)),2},
							  {((void**)(&imu_command_pdo)),2}};

ecat_pdo_entry_t imu_tx_pdos[] = {{((void**)(&imu_medulla_id_pdo)),1},
							  {((void**)(&imu_current_state_pdo)),1},
							  {((void**)(&imu_medulla_counter_pdo)),1},
							  {((void**)(&imu_error_flags_pdo)),1},
							  {((void**)(&XAngDelta_pdo)),4},
							  {((void**)(&XAccel_pdo)),4},
							  {((void**)(&YAngDelta_pdo)),4},
							  {((void**)(&YAccel_pdo)),4},
							  {((void**)(&ZAngDelta_pdo)),4},
							  {((void**)(&ZAccel_pdo)),4},
							  {((void**)(&Status_pdo)),1},
							  {((void**)(&Seq_pdo)),1},
							  {((void**)(&Temp_pdo)),2}};

void imu_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter,TC0_t *timestamp_timer, uint16_t **master_watchdog) {

	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_IMU_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_IMU_INPUTS_SIZE, 0x2000);
	ecat_configure_pdo_entries(ecat_slave, imu_rx_pdos, MEDULLA_IMU_RX_PDO_COUNT, imu_tx_pdos, 19);   // TODO: What's 19?

	kvh_init(&PORTF, &USARTF0, &kvh_data, kvh_data_pdo);

	*master_watchdog = imu_counter_pdo;
	*packet_counter = imu_medulla_counter_pdo;
	*imu_medulla_id_pdo = id;
	*commanded_state = imu_command_state_pdo;
	*current_state = imu_current_state_pdo;
}

void imu_read_data(ecat_slave_t ecat_port){
	   // Init debug UART
	   uart_debug = uart_init_port(&PORTE, &USARTE0, uart_baud_921600, imu_tx_buffer, 10, imu_rx_buffer, 100);
	   uart_connect_port(&uart_debug, true);

		// reset updata mode
		//imu_reset_update_mode();

		// set to continuous mode
		uint8_t response;
		//response = imu_set_continuous_mode();

		// unset continuous mode
		//response = imu_unset_continuous_mode();


		// stop continuous mode
		//response = imu_set_active_mode();

		// soft reset
		//imu_soft_reset();

	    uint32_t pResponse[36];
		uint8_t u8_pResponse[36];
		uint32_t *roll;
		uint32_t *pitch;
		uint32_t *yaw;

		uint8_t led_cnt = 0;

		while (1)
		{
			// Blinking means IMU is performing fine.
			if (led_cnt/((uint32_t)(100)))
			{
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_GREEN;
			}
			else
			{
				LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_BLUE;
			}
			++led_cnt;

			uint32_t ptr = 0;

			(*imu_medulla_id_pdo) = 0;
			(*imu_current_state_pdo)= 0;
			(*imu_medulla_counter_pdo)= 0;
			(*imu_error_flags_pdo)= 0;
			(*XAngDelta_pdo) = 0;
			(*XAccel_pdo)= 0;
			(*YAngDelta_pdo)= 0;
			(*YAccel_pdo)= 0;
			(*ZAngDelta_pdo)= 0;
			(*ZAccel_pdo)= 0;
			(*Status_pdo)= 0;
			(*Seq_pdo)= 0;
			(*Temp_pdo)= 0;

			//if (*imu_command_pdo)
			if (1)
			{
				// Read data
				//response = imu_read_accel_angRate_OriMatrix(u8_pResponse);
				response = imu_pull_accel_angRate_OriMatrix(u8_pResponse);

				if (response)
				{
					//_delay_ms(1);


					for (uint8_t i=0;i<36;i++)
					{
						pResponse[i] = u8_pResponse[i];
					}


						//TxPDO entries
					(*imu_medulla_id_pdo) = 1;
					(*imu_current_state_pdo) = 2;
					(*imu_medulla_counter_pdo) = 3;
					(*imu_error_flags_pdo) = 4;

					// Populate data from IMU. Refer to p. 10 in manual for
					// data locations.
					uint8_t *ptr;

					// XAngDelta
					ptr = u8_pResponse+4;
					populate_byte_to_data(ptr,XAngDelta_pdo);

					// YAngDelta
					ptr = u8_pResponse+8;
					populate_byte_to_data(ptr,YAngDelta_pdo);


					// ZAngDelta
					ptr = u8_pResponse+12;
					populate_byte_to_data(ptr,ZAngDelta_pdo);

					// XAccel
					ptr = u8_pResponse+16;
					populate_byte_to_data(ptr,XAccel_pdo);

					// YAccel
					ptr = u8_pResponse+20;
					populate_byte_to_data(ptr,YAccel_pdo);

					// ZAccel
					ptr = u8_pResponse+24;
					populate_byte_to_data(ptr,ZAccel_pdo);

					// Status
					ptr = u8_pResponse+28;
					populate_byte_to_data(ptr,Status_pdo);

					// Seq
					ptr = u8_pResponse+29;
					populate_byte_to_data(ptr,Seq_pdo);

					// Temp
					ptr = u8_pResponse+30;
					populate_byte_to_data(ptr,Temp_pdo);
				}
			}

		}
		//Read new commands from the ethercat slave
		ecat_read_rx_sm(&ecat_port);
		//Send the new sensor data to the ethercat slave
		ecat_write_tx_sm(&ecat_port);

		//_delay_ms(15);
	}


	//_delay_ms(50);


}

bool imu_calculating_checksum(uint8_t *rx_buffer,uint8_t rx_buffer_length){
	bool tGoodResponse;
	uint8_t tix;
	uint16_t tChksum;
	uint16_t tResponseChksum;

	tChksum = 0;
	for (tix = 0;tix < (rx_buffer_length-2);tix++)
	{
		tChksum += (uint16_t)(rx_buffer[tix]);
	}

	tResponseChksum = 0;
	tResponseChksum = rx_buffer[rx_buffer_length-2] << 8;
	tResponseChksum += rx_buffer[rx_buffer_length-1];

	if (tChksum==tResponseChksum)
	{
		tGoodResponse = true;
	}
	else
	{
		tGoodResponse = false;
	}

	return tGoodResponse;
}

uint8_t imu_set_continuous_mode(void){
		unsigned char tx_buffer[4] = {0xC4,0xC1,0x29,0xC8};
		unsigned char rx_buffer[8];
		uint8_t response;
		uint8_t tx_buffer_length;


		while (1)
		{
			tx_buffer_length = 0;
			uart_tx_data(&uart_debug,tx_buffer, 4);
			uart_rx_data(&uart_debug,rx_buffer,8);
			//_delay_ms(5);

			if ((rx_buffer[0] == 0xC4))
			{
				return rx_buffer[0];
			}

		}

}

uint8_t imu_unset_continuous_mode(void){
		uint8_t tx_buffer[3];
		uint8_t rx_buffer[1];
		uint8_t response;
		uint8_t tx_buffer_length;

		tx_buffer[0] = 0xFA;
		tx_buffer[1] = 0x75;
		tx_buffer[2] = 0xB4;

		//while (1)
		//{
			tx_buffer_length = 0;
			tx_buffer_length = uart_tx_data(&uart_debug,tx_buffer, 3);
			uart_rx_data(&uart_debug,rx_buffer,1);


			//if ((rx_buffer[0] == 0xC4))
			//{
				return rx_buffer[0];
			//}

		//}

}


uint8_t imu_set_active_mode(void){
		uint8_t tx_buffer[4];
		uint8_t rx_buffer[4];

		tx_buffer[0] = 0xD4;
		tx_buffer[1] = 0xA3;
		tx_buffer[2] = 0x47;
		tx_buffer[3] = 0x01;

		while (1)
		{
			uart_tx_data(&uart_debug,tx_buffer, 4);
			uart_rx_data(&uart_debug,rx_buffer,4);
			_delay_ms(5);
		}

}

uint8_t imu_read_euler_angles(uint32_t *ptr_roll, uint32_t *ptr_pitch, uint32_t *ptr_yaw){
	uint8_t tx_buffer[1];
		uint8_t rx_buffer[19];

		tx_buffer[0] = 0xCE;


		uart_tx_data(&uart_debug,tx_buffer, 1);
		uart_rx_data(&uart_debug,rx_buffer,19);

		// read euler angles

		(*ptr_roll) = SHIFT_3BYTE(rx_buffer[1]);
		(*ptr_roll) += SHIFT_2BYTE(rx_buffer[2]);
		(*ptr_roll) += SHIFT_1BYTE(rx_buffer[3]);
		(*ptr_roll) += rx_buffer[4];

		(*ptr_pitch) = SHIFT_3BYTE(rx_buffer[5]);
		(*ptr_pitch) += SHIFT_2BYTE(rx_buffer[6]);
		(*ptr_pitch) += SHIFT_1BYTE(rx_buffer[7]);
		(*ptr_pitch) += rx_buffer[8];

		(*ptr_yaw) = SHIFT_3BYTE(rx_buffer[9]);
		(*ptr_yaw) += SHIFT_2BYTE(rx_buffer[10]);
		(*ptr_yaw) += SHIFT_1BYTE(rx_buffer[11]);
		(*ptr_yaw) += rx_buffer[12];



		return rx_buffer[0];
}


uint8_t imu_read_accel_angRate_OriMatrix(uint8_t *ptr_response){
		uint8_t tx_buffer[1];
		uint8_t rx_buffer[67];
		uint8_t goodResponse;
		tx_buffer[0] = 0x00;
		uint8_t rx_buffer_length;

		while (1)
		{
			//uart_tx_data(&uart_debug,tx_buffer, 1);
			rx_buffer_length = uart_rx_data(&uart_debug,rx_buffer,67);

			// Read good package
			//goodResponse = 0;
			//if (rx_buffer[0]==0xC8)
			//{
				//goodResponse  = imu_calculating_checksum(rx_buffer,67);
			//}

			// stop and return if good package detected
			if (rx_buffer[0]==0xC8)
			{
				for (int i=0;i<67;i++)
				{
					*(ptr_response+i) = rx_buffer[i];
				}
				return rx_buffer[0];
			}

		}
}



uint8_t imu_pull_accel_angRate_OriMatrix(uint8_t *ptr_response){
		uint8_t tx_buffer[1];
		uint8_t rx_buffer[67];
		uint8_t buffer[67];

		int package_length = 67;
		bool goodResponse;
		int rx_buffer_length;
		int tx_buffer_length;

		uint16_t cnt = 0;
		uint16_t offset = 0;

		tx_buffer[0] = 0xC8;
		while (1)
		{
			for (uint8_t i=0;i<67;i++)
			{
				buffer[i] = 0;
			}

			// send command
			tx_buffer_length = uart_tx_data(&uart_debug,tx_buffer,1);

			// grab data
			rx_buffer_length = 0;
			cnt = 0;
			while (rx_buffer_length<67)
			{
				offset = uart_rx_data(&uart_debug,rx_buffer,67);

				for (int i=0;i<offset;i++)
				{
					buffer[cnt] = rx_buffer[i];
					++cnt;
				}
				rx_buffer_length = rx_buffer_length + offset;
			}


			 //Read good package
			goodResponse = 0;
			if ((buffer[0])==200)
			{
				goodResponse  = imu_calculating_checksum(buffer,67);
			}
			//
			// stop and return if good package detected
			if (goodResponse)
			{
				for (int i=0;i<67;i++)
				{
					*(ptr_response+i) = buffer[i];
				}
				return buffer[0];
			}
		}
}





uint8_t imu_soft_reset(void){
	uint8_t tx_buffer[3];
	uint8_t rx_buffer[1];

	tx_buffer[0] = 0xFE;
	tx_buffer[1] = 0x9E;
	tx_buffer[2] = 0x3A;


	while (1)
	{
		uart_tx_data(&uart_debug,tx_buffer, 3);
		uart_rx_data(&uart_debug,rx_buffer,1);

		//if (rx_buffer[0]==0xC8)
		//{
			//for (int i=0;i<67;i++)
			//{
				//*(ptr_response+i) = rx_buffer[i];
			//}
			//return rx_buffer[0];
		//}

	}
}

uint8_t imu_reset_update_mode(void){
	uint8_t tx_buffer[1];
	uint8_t rx_buffer[1];

	tx_buffer[0] = 0x32;

	while (1)
	{
		uart_tx_data(&uart_debug,tx_buffer, 1);
		uart_rx_data(&uart_debug,rx_buffer,1);

		//if (rx_buffer[0]==0xC8)
		//{
			//for (int i=0;i<67;i++)
			//{
				//*(ptr_response+i) = rx_buffer[i];
			//}
			//return rx_buffer[0];
		//}

	}
}

void kvh_init(PORT_t *port, USART_t *uart, kvh_data_t *data, uint8_t *data_buffer) {
	data->uart_port = uart_init_port(port, uat, uart_baud_921600, data->tx_buffer, KVH_TX_BUFFER_LENGTH, data->rx_buffer, KVH_RX_BUFFER_LENGTH);
	uart_connect_port(&(data->uart_port), 0);
	data->data_buffer = data_buffer;
}

void populate_byte_to_data(const uint8_t* data_byte,uint32_t* data){
	(*data) = SHIFT_3BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += SHIFT_2BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += SHIFT_1BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += (uint32_t)(*data_byte);

}
