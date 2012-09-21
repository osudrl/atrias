#include "atrias_elabs_conn/ELabsConn.h"
namespace atrias {
namespace eLabsConn {
ELabsConn::ELabsConn(std::string name) : TaskContext(name),
	newStateCallback("newStateCallback") {
	
	this->provides("connector")
	    ->addOperation("sendControllerOutput", &ELabsConn::sendControllerOutput, this, RTT::ClientThread);
	this->requires("rtOps")
	    ->addOperationCaller(newStateCallback);
	
	rt_fd   = -1;
	counter = 0;
	lLegA   = NULL;
	lLegB   = NULL;
}

bool ELabsConn::configureHook() {
	RTT::TaskContext *peer = this->getPeer("atrias_rt");
	if (!peer) {
		log(RTT::Error) << "[NoopConn] Failed to connect to RTOps!" << RTT::endlog();
		return false;
	}
	newStateCallback = peer->provides("rtOps")->getOperation("newStateCallback");
	
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
	
	LEG_MEDULLA_DEFS(0);
	LEG_MEDULLA_DEFS(1);
	if (ecrt_slave_config_pdos(sc0, 4, slave_0_syncs) || ecrt_slave_config_pdos(sc1, 4, slave_1_syncs)) {
		log(Error) << "ecrt_slave_config_pdos failed!" << endlog();
		ecrt_release_master(ec_master);
		return false;
	}
	
	LEG_MEDULLA_REG_PDOS(0);
	LEG_MEDULLA_REG_PDOS(1);
	
	RTT::os::TimeService::nsecs cur_time;
	cur_time = RTT::os::TimeService::Instance()->getNSecs();
	ecrt_master_application_time(ec_master, EC_NSECS_TO_NANO(cur_time));
	ecrt_slave_config_dc(sc0, 0x0300, LOOP_PERIOD_NS, LOOP_PERIOD_NS - ((cur_time + LOOP_OFFSET_NS) % LOOP_PERIOD_NS), 0, 0);
	ecrt_slave_config_dc(sc1, 0x0300, LOOP_PERIOD_NS, LOOP_PERIOD_NS - ((cur_time + LOOP_OFFSET_NS) % LOOP_PERIOD_NS), 0, 0);
	
	return true;
}

bool ELabsConn::startHook() {
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
	
	uint8_t* domain_pd = ecrt_domain_data(domain);
	
	printf("Creating leg medulla objects\n");
	LEG_MEDULLA_CREATE(lLegA, 0);
	LEG_MEDULLA_CREATE(lLegB, 1);
	
	//rtos_enable_rt_warning();
	inOp = false;
	
	return true;
}

void ELabsConn::updateHook() {
	RTT::os::MutexLock lock(eCatLock);
	RTT::os::TimeService::nsecs cur_time;
	cur_time = RTT::os::TimeService::Instance()->getNSecs();
	ecrt_rtdm_master_application_time(rt_fd, EC_NSECS_TO_NANO(cur_time));
	if (++counter >= 10) {
		counter = 0;
		ecrt_rtdm_master_sync_reference_clock(rt_fd);
	}
	ecrt_rtdm_master_sync_slave_clocks(rt_fd);
	ecrt_rtdm_domain_queque(rt_fd);
	ecrt_rtdm_master_send(rt_fd);
	rt_task_sleep(300000);
	ecrt_rtdm_master_recieve(rt_fd);
	ecrt_rtdm_domain_process(rt_fd);
	if (!inOp) {
		ec_master_state_t master_state;
		ecrt_rtdm_master_state(rt_fd, &master_state);
		if (master_state.al_states == ELABS_OP_STATE) {
			inOp = true;
			lLegA->postOpInit();
			lLegB->postOpInit();
		}
	} else {
		lLegA->processReceiveData(robotState);
		lLegB->processReceiveData(robotState);
	}
	newStateCallback(robotState);
}

void ELabsConn::sendControllerOutput(atrias_msgs::controller_output controller_output) {
	if (!lLegA || !lLegB)
		return;
	RTT::os::MutexLock lock(eCatLock);
	lLegA->processTransmitData(controller_output);
	lLegB->processTransmitData(controller_output);
	ecrt_rtdm_domain_queque(rt_fd);
	ecrt_rtdm_master_send(rt_fd);
}

void ELabsConn::stopHook() {
	rtos_disable_rt_warning();
}

void ELabsConn::cleanupHook() {
	ecrt_release_master(ec_master);
}

ORO_CREATE_COMPONENT(ELabsConn)
}
}
