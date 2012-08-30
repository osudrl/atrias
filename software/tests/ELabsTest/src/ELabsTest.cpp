#include "atrias_elabs_test/ELabsTest.h"

void sig_handler(int signum) {
	return;
}

ELabsTest::ELabsTest(std::string name) : TaskContext(name) {
	rt_fd   = -1;
	counter = 0;
	signal(SIGXCPU, sig_handler);
}

bool ELabsTest::configureHook() {
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		log(RTT::Error) << "[RTOps] Failed to lock memory!" << RTT::endlog();
		return false;
	}
	
	MstrAttach.masterindex = 0;
	ec_master = ecrt_request_master(MstrAttach.masterindex);
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
	
	ec_slave_config_t* sc0 = ecrt_master_slave_config(ec_master, 0, 0, VENDOR_ID, PRODUCT_CODE0);
	ec_slave_config_t* sc1 = ecrt_master_slave_config(ec_master, 0, 1, VENDOR_ID, PRODUCT_CODE1);
	if (!sc0 || !sc1) {
		log(Error) << "ecrt_master_slave_config returned NULL!" << endlog();
		ecrt_release_master(ec_master);
		return false;
	}
	
	if (ecrt_slave_config_pdos(sc0, 4, slave_0_syncs) || ecrt_slave_config_pdos(sc1, 4, slave_1_syncs)) {
		log(Error) << "ecrt_slave_config_pdos failed!" << endlog();
		ecrt_release_master(ec_master);
		return false;
	}
	
	unsigned int command_off;
	unsigned int current_off;
	unsigned int timestep_off;
	unsigned int encoder_off;
	ec_pdo_entry_reg_t entry_regs[] = {
		{0, 0, VENDOR_ID, PRODUCT_CODE0, 0x5, 0x1,  &command_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE0, 0x3, 0x2,  &current_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE0, 0x6, 0x1, &timestep_off, NULL},
		{0, 0, VENDOR_ID, PRODUCT_CODE0, 0x7, 0x2,  &encoder_off, NULL},
		{0, 0,      0x00,         0x00, 0x0, 0x0,          NULL, NULL}
	};
	
	if (ecrt_domain_reg_pdo_entry_list(domain, entry_regs)) {
		log(Error) << "ecrt_domain_reg_pdo_entry_list" << endlog();
		ecrt_release_master(ec_master);
		return false;
	}
	
	timespec cur_time;
	clock_gettime(CLOCK_REALTIME, &cur_time);
	ecrt_master_application_time(ec_master, EC_NEWTIMEVAL2NANO(cur_time));
	ecrt_slave_config_dc(sc0, 0x0300, LOOP_PERIOD_NS, LOOP_PERIOD_NS - ((cur_time.tv_nsec + LOOP_OFFSET_NS) % LOOP_PERIOD_NS), 0, 0);
	ecrt_slave_config_dc(sc1, 0x0300, LOOP_PERIOD_NS, LOOP_PERIOD_NS - ((cur_time.tv_nsec + LOOP_OFFSET_NS) % LOOP_PERIOD_NS), 0, 0);
	
	return true;
}

bool ELabsTest::startHook() {
	rt_fd = rt_dev_open("ec_rtdm0", 0);
	if (rt_fd < 0) {
		log(Error) << "rt_dev_open failed! Error: " << errno << endlog();
		return false;
	}
	
	MstrAttach.domainindex = ecrt_domain_index(domain);
	if (ecrt_rtdm_master_attach(rt_fd, &MstrAttach) < 0) {
		log(Error) << "ecrt_rtdm_master_attach failed!" << endlog();
		return false;
	}
	
	if (ecrt_master_activate(ec_master)) {
		log(Error) << "ecrt_master_activate() failed!" << endlog();
		return false;
	}
	
	//rtos_enable_rt_warning();
	
	return true;
}

void ELabsTest::updateHook() {
	rtos_enable_rt_warning();
	timespec cur_time;
	clock_gettime(CLOCK_REALTIME, &cur_time);
	ecrt_master_application_time(ec_master, EC_NEWTIMEVAL2NANO(cur_time));
	if (++counter >= 10) {
		counter = 0;
		ecrt_rtdm_master_sync_reference_clock(rt_fd);
	}
	ecrt_rtdm_master_sync_slave_clocks(rt_fd);
	ecrt_rtdm_domain_queque(rt_fd);
	ecrt_rtdm_master_send(rt_fd);
	timespec delay = {
		0,
		300000
	};
	clock_nanosleep(CLOCK_MONOTONIC, 0, &delay, NULL);
	ecrt_rtdm_master_recieve(rt_fd);
	ecrt_rtdm_domain_process(rt_fd);
}

void ELabsTest::stopHook() {
	rtos_disable_rt_warning();
}

void ELabsTest::cleanupHook() {
	ecrt_release_master(ec_master);
}

ORO_CREATE_COMPONENT(ELabsTest)
