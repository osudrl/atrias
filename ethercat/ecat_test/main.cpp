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
	
	ec_slave_config_t* sc = ecrt_master_slave_config(ec_master, 0, 0, VENDOR_ID, PRODUCT_CODE);
	
	if (ecrt_slave_config_pdos(sc, 4, slave_0_syncs)) {
		perror("ecrt_slave_config_pdos() failed.");
		ecrt_release_master(ec_master);
		return 3;
	}
	
	unsigned int command_off;
	unsigned int current_off;
	unsigned int timestep_off;
	unsigned int encoder_off;
	ec_pdo_entry_reg_t entry_regs[5] = { 
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x5, 0x1,  &command_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x3, 0x2,  &current_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x6, 0x1, &timestep_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x7, 0x2,  &encoder_off, NULL},
		{0, 0,      0x00,         0x00, 0x0, 0x0,          NULL, NULL}};
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
	uint16_t*      current  = (uint16_t*)      (domain_pd + current_off);
	uint16_t*      timestep = (uint16_t*)      (domain_pd + timestep_off);
	uint32_t*      encoder  = (uint32_t*)      (domain_pd + encoder_off);
	
	(*command) = 0;
	while (!done) {
		ecrt_master_receive(ec_master);
		ecrt_domain_process(domain);
		(*command)++;
		printf("command: %3u, timestep: %3u, encoder: %8u\n", (unsigned int) *command, *timestep, (unsigned int) *encoder);
		ecrt_domain_queue(domain);
		ecrt_master_send(ec_master);
		usleep(10000);
	}
	
	ecrt_release_master(ec_master);
	return 0;
}
