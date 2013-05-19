#include "../include/medulla_imu.h" 
#include "../include/medulla.h"
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

uint32_t *XAngRate_pdo;
uint32_t *XAccel_pdo;
uint32_t *YAngRate_pdo;
uint32_t *YAccel_pdo;
uint32_t *ZAngRate_pdo;
uint32_t *ZAccel_pdo;
uint32_t *M11_pdo;
uint32_t *M12_pdo;
uint32_t *M13_pdo;
uint32_t *M21_pdo;
uint32_t *M22_pdo;
uint32_t *M23_pdo;
uint32_t *M31_pdo;
uint32_t *M32_pdo;
uint32_t *M33_pdo;

void populate_byte_to_data(const uint8_t* data_byte,uint32_t* data);

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
									
//ecat_pdo_entry_t imu_tx_pdos[] = {{((void**)(&M11_pdo)),4},
								//{((void**)(&M12_pdo)),4},
								//{((void**)(&M13_pdo)),4},
								//{((void**)(&M21_pdo)),4},
								//{((void**)(&M22_pdo)),4},
								//{((void**)(&M23_pdo)),4},
								//{((void**)(&M31_pdo)),4},
								//{((void**)(&M32_pdo)),4},
								//{((void**)(&M33_pdo)),4}};	
//
void imu_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter,TC0_t *timestamp_timer, uint16_t **master_watchdog) {

	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, 5, 0x1000, tx_sm_buffer, 64, 0x2000);
	ecat_configure_pdo_entries(ecat_slave, imu_rx_pdos, 3, imu_tx_pdos, 19);
	
	//ecat_init_sync_managers(ecat_slave, rx_sm_buffer, 5, 0x1000, tx_sm_buffer, 36, 0x2000);
	//ecat_configure_pdo_entries(ecat_slave, imu_rx_pdos, 3, imu_tx_pdos, 9); 	
		//
	*master_watchdog = imu_counter_pdo;
	*packet_counter = imu_medulla_counter_pdo;
	*imu_medulla_id_pdo = id;
	*commanded_state = imu_command_state_pdo;
	*current_state = imu_current_state_pdo;
}

void imu_read_data(ecat_slave_t ecat_port){
	
	   // Read raw data through UART from IMU
	   int tx_byte_cnt;
	   int rx_byte_cnt;
	   uint8_t rx_byte;
	   uint8_t byte;
	   
	   // init UART
	   uart_debug = uart_init_port(&PORTE, &USARTE0, uart_baud_460800, imu_tx_buffer, 10, imu_rx_buffer, 100);
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
		
	    uint32_t pResponse[67];
		uint8_t u8_pResponse[67];
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
			(*XAngRate_pdo) = 0;
			(*XAccel_pdo)= 0;
			(*YAngRate_pdo)= 0;
			(*YAccel_pdo)= 0;
			(*ZAngRate_pdo)= 0;
			(*ZAccel_pdo)= 0;
			(*M11_pdo)= 0;
			 (*M12_pdo)= 0;
			(*M13_pdo)= 0;
			(*M21_pdo)= 0;
			(*M22_pdo)= 0;
			(*M23_pdo)= 0;
			(*M31_pdo)= 0;
			(*M32_pdo)= 0;
			(*M33_pdo)= 0;

			//if (*imu_command_pdo)
			if (1)
			{
				// Read data
				//response = imu_read_accel_angRate_OriMatrix(u8_pResponse);						 				
				response = imu_pull_accel_angRate_OriMatrix(u8_pResponse);
				
				if (response)
				{
					//_delay_ms(1);
					 
					 
					for (uint8_t i=0;i<67;i++)
					{
						pResponse[i] = u8_pResponse[i];
					}
					 
					 
						//TxPDO entries
					(*imu_medulla_id_pdo) = 1;
					(*imu_current_state_pdo) = 2;
					(*imu_medulla_counter_pdo) = 3;
					(*imu_error_flags_pdo) = 4;
			
					uint8_t *ptr;
					
					// XAccel	
					ptr = u8_pResponse+1;			
					populate_byte_to_data(ptr,XAccel_pdo);
					
					// YAccel
					ptr = u8_pResponse+5;			
					populate_byte_to_data(ptr,YAccel_pdo);
					
					// ZAccel
					ptr = u8_pResponse+9;			
					populate_byte_to_data(ptr,ZAccel_pdo);
					
					// XAngRate
					ptr = u8_pResponse+13;			
					populate_byte_to_data(ptr,XAngRate_pdo);
					
					// YAngRate
					ptr = u8_pResponse+17;			
					populate_byte_to_data(ptr,YAngRate_pdo);
					
			
					// ZAngRate
					ptr = u8_pResponse+21;			
					populate_byte_to_data(ptr,ZAngRate_pdo);
					
					
					// M11
					byte = 25;
					ptr = u8_pResponse+25;			
					populate_byte_to_data(ptr,M11_pdo);
					
					// M12
					ptr = u8_pResponse+29;			
					populate_byte_to_data(ptr,M12_pdo);
					
					// M13
					ptr = u8_pResponse+33;			
					populate_byte_to_data(ptr,M13_pdo);
					
				
					// M21
					ptr = u8_pResponse+37;			
					populate_byte_to_data(ptr,M21_pdo);
					
				
					// M22
					ptr = u8_pResponse+41;			
					populate_byte_to_data(ptr,M22_pdo);
					

					// M23
					ptr = u8_pResponse+45;			
					populate_byte_to_data(ptr,M23_pdo);
					
					// M31
					ptr = u8_pResponse+49;			
					populate_byte_to_data(ptr,M31_pdo);
					
					
					// M32
					ptr = u8_pResponse+53;			
					populate_byte_to_data(ptr,M32_pdo);
					

					// M33
					ptr = u8_pResponse+57;			
					populate_byte_to_data(ptr,M33_pdo);
					
				//
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

void populate_byte_to_data(const uint8_t* data_byte,uint32_t* data){
	(*data) = SHIFT_3BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += SHIFT_2BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += SHIFT_1BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += (uint32_t)(*data_byte);

}
