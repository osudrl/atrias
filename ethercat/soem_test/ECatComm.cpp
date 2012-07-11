#include "atrias_ecat_master/ECatComm.h"

ECatComm::ECatComm() {
	recv_callback = NULL;
	data          = new RobotComm();
	initialized   = false;
}

void ECatComm::transmitHook() {
	if (!initialized) return;
	ec_send_processdata();
}

void ECatComm::cyclicTask() {
	if (!initialized) return;
	// Is SOEM thread-safe? I've sent a message to the mailinglist... I hope to receive
	// a reply today.
	
	// This first call clears out the return packet from transmitHook()'s frame.
	ec_receive_processdata(0);
	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUT_US);
	
	// Notify the Orocos stuff that we have received data;
	if (recv_callback != NULL) recv_callback();
}

RobotComm* ECatComm::getData() {
	return data;
}

void ECatComm::registerDataCallback(void (*dataCallback)()) {
	recv_callback = dataCallback;
}

void ECatComm::init() {
	if (!ec_init("eth0")) {
		printf("%s: ec_init() failed!", __FILE__);
		return;
	}
	
	ec_config_init(FALSE);
	
	printf("%d slaves identified", ec_slavecount);
	
	// What does this do anyway? Copied from simple_test.c...
	ec_config_map(&IOmap);
	ec_configdc();
	
	// Wait for SAFE-OP
	ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
	
	// Send them to OP
	for (int i = 0; i < ec_slavecount; i++) {
		ec_slave[i].state = EC_STATE_OPERATIONAL;
	}
	// simple_test.c says sending and processing a frame cycle can help here.
	// Doesn't seem necessary, though (see ebox.c) -- maybe it's for slaves w/ direct I/O?
	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUTRET);
	ec_writestate(0);
	ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);
	// We are now in OP
	
	for (int i = 0; i < ec_slavecount; i++) {
		// We can configure the DC shift time here.
		// Also, SOEM likes 1-indexing slave positions...
		ec_dcsync0(i + 1, TRUE, LOOP_PERIOD_NS, 0);
		
		switch (ec_slave[i].eep_id) {
			case TEST_PRODUCT_CODE:
				if (data->test1 == NULL) free(data->test1);
				data->test1 = new TestMedulla(&(ec_slave[i]));
				break;
		}
	}
	
	initialized = true;
}
