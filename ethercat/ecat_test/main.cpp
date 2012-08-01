#include "main.h"

void sig_handler(int signum) {
	done = true;
}

int main(int argc, char ** argv) {
	signal(SIGINT, sig_handler);
	ec_master_t* ec_master = ecrt_request_master(0);
	if (ec_master == NULL) {
		perror("ecrt_request_master failed.");
		return 1;
	}
	ec_domain_t* domain = ecrt_master_create_domain (ec_master);
	if (domain == NULL) {
		perror("ecrt_master_create_domain failed.");
		ecrt_release_master(ec_master);
		return 2;
	}
	
	ec_slave_config_t* sc0 = ecrt_master_slave_config(ec_master, 0, 0, VENDOR_ID, PRODUCT_CODE);
	
	if (ecrt_slave_config_pdos(sc0, 4, slave_0_syncs)) {
		perror("ecrt_slave_config_pdos() failed.");
		ecrt_release_master(ec_master);
		return 3;
	}
	
	unsigned int command_off;
	unsigned int counter_off;
	ec_pdo_entry_reg_t entry_regs[] = { 
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x5, 0x1,  &command_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x6, 0x1,  &counter_off, NULL},
		{0, 0,      0x00,         0x00, 0x0, 0x0,           NULL, NULL}};
	if (ecrt_domain_reg_pdo_entry_list(domain, entry_regs)) {
		perror("ecrt_domain_reg_pdo_entry_list() failed");
		ecrt_release_master(ec_master);
		return 4;
	}
	if (ecrt_master_activate(ec_master)) {
	perror("ecrt_master_activate() failed");
	ecrt_release_master(ec_master);
	return 5;
}	

	uint8_t* domain_pd = ecrt_domain_data(domain);
	
	unsigned char* command  = (unsigned char*) (domain_pd + command_off);
	uint16_t*      counter  = (uint16_t*)      (domain_pd + counter_off);
	
	(*command) = 0;
	char input;
		timespec cur_time;
	clock_gettime(CLOCK_REALTIME, &cur_time);
	ecrt_master_application_time(ec_master, EC_NEWTIMEVAL2NANO(cur_time));
	ecrt_slave_config_dc(sc0, 0x0300, LOOP_PERIOD_NS, LOOP_PERIOD_NS - (cur_time.tv_nsec % LOOP_PERIOD_NS), 0, 0);
//	ecrt_slave_config_dc(sc1, 0x0300, LOOP_PERIOD_NS, LOOP_PERIOD_NS - (cur_time.tv_nsec % LOOP_PERIOD_NS), 0, 0);
	
	long tgt_time = cur_time.tv_nsec;
	timespec wait_time = {
		0,
		0
	};
	ec_master_state_t master_state;
	


	while (!done) {
		*command = 2;
//		if (*counter < 20000)
		*counter += 1;
		printf("command: %d\n",*command);
		//printf("command: %3u, timestep: %3u, encoder: %8u\n", (unsigned int) *command, *timestep, (unsigned int) *encoder);
		
		clock_gettime(CLOCK_REALTIME, &cur_time);
		ecrt_master_application_time(ec_master, EC_NEWTIMEVAL2NANO(cur_time));
		ecrt_master_sync_reference_clock(ec_master);
		ecrt_master_sync_slave_clocks(ec_master);
		ecrt_domain_queue(domain);
		ecrt_master_send(ec_master);
		usleep(10);
		ecrt_master_receive(ec_master);
		ecrt_domain_process(domain);
		wait_time.tv_nsec = LOOP_PERIOD_NS - (cur_time.tv_nsec % LOOP_PERIOD_NS);
		nanosleep(&wait_time, NULL);
	}
	
	ecrt_release_master(ec_master);
	return 0;
}
