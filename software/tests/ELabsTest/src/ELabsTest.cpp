#include "atrias_elabs_test/ELabsTest.h"

ELabsTest::ELabsTest(std::string name) : TaskContext(name) {
	return;
}

bool ELabsTest::configureHook() {
	ec_master = ecrt_request_master(0);
	if (!ec_master) {
		log(Error) << "ecrt_request_master() returned NULL!" << endlog();
		return false;
	}
	
	domain = ecrt_master_create_domain (ec_master);
	if (!domain) {
		log(Error) << "ecrt_master_create_domain() returned NULL!" << endlog();
		ecrt_release_master(ec_master);
		return false;
	}
	
	ec_slave_config_t* sc0 = ecrt_master_slave_config(ec_master, 0, 0, VENDOR_ID, PRODUCT_CODE);
	if (!sc0) {
		log(Error) << "ecrt_master_slave_config returned NULL!" << endlog();
		ecrt_release_master(ec_master);
		return false;
	}
	
	if (ecrt_slave_config_pdos(sc0, 4, slave_0_syncs)) {
		log(Error) << "ecrt_slave_config_pdos failed!" << endlog();
		ecrt_release_master(ec_master);
		return false;
	}
	
	unsigned int command_off;
	unsigned int current_off;
	unsigned int timestep_off;
	unsigned int encoder_off;
	ec_pdo_entry_reg_t entry_regs[] = {
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x5, 0x1,  &command_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x3, 0x2,  &current_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x6, 0x1, &timestep_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x7, 0x2,  &encoder_off, NULL},
		{0, 0,      0x00,         0x00, 0x0, 0x0,          NULL, NULL}
	};
	
	if (ecrt_domain_reg_pdo_entry_list(domain, entry_regs)) {
		log(Error) << "ecrt_domain_reg_pdo_entry_list" << endlog();
		ecrt_release_master(ec_master);
		return false;
	}
	
	return true;
}

bool ELabsTest::startHook() {
	if (ecrt_master_activate(ec_master)) {
		log(Error) << "ecrt_master_activate() failed!" << endlog();
		return false;
	}
	
	return true;
}

void ELabsTest::updateHook() {
	ecrt_domain_queue(domain);
	ecrt_master_send(ec_master);
	timespec delay = {
		0,
		300000
	};
	nanosleep(&delay, NULL);
	ecrt_master_receive(ec_master);
	ecrt_domain_process(domain);
}

void ELabsTest::cleanupHook() {
	ecrt_release_master(ec_master);
}

ORO_CREATE_COMPONENT(ELabsTest)
