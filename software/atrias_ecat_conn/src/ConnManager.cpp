#include "atrias_ecat_conn/ConnManager.h"

namespace atrias {

namespace ecatConn {

ConnManager::ConnManager(ECatConn* ecat_conn) :
             RTT::Activity(80) {
	eCatConn = ecat_conn;
	return;
}

ConnManager::~ConnManager() {
	ec_slave[0].state = EC_STATE_INIT;
	ec_writestate(0);
	ec_close();
}

inline void ConnManager::cycleECat() {
	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUT_US);
}

inline void ConnManager::nanosleep(RTT::os::TimeService::nsecs sleeptime) {
	timespec delay = {
		0,
		(long) sleeptime
	};
	clock_nanosleep(CLOCK_MONOTONIC, 0, &delay, NULL);
}

bool ConnManager::configure() {
	if (!ec_init("eth0")) {
		log(RTT::Error) << "[ECatConn] ConnManager: ec_init() failed!" << RTT::endlog();
		return false;
	}
	
	ec_config_init(FALSE);
	
	log(RTT::Info) << "[ECatConn] " << ec_slavecount << " EtherCAT slaves identified." << RTT::endlog();
	if (ec_slavecount < 1) {
		log(RTT::Error) << "[ECatConn] Failed to identify any slaves! Failing to init." << RTT::endlog();
		return false;
	}
	
	ec_configdc();
	
	ec_config_map(IOmap);
	
	// Wait for SAFE-OP
	ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
	
	return true;
}

bool ConnManager::initialize() {
	// Send the EtherCAT slaves into OP
	ec_slave[0].state = EC_STATE_OPERATIONAL;
	
	// This might not even be necessary... should test.
	cycleECat();
	
	ec_writestate(0);
	ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);
	
	// We are now in OP.
	// Configure the distributed clocks for each slave.
	for (int i = 1; i <= ec_slavecount; i++) {
		ec_dcsync0(i, true, CONTROLLER_LOOP_PERIOD_NS,
		           CONTROLLER_LOOP_PERIOD_NS - CONTROLLER_LOOP_OFFSET_NS);
	}
	
	// Update ec_DCtime so we can calculate stop time below.
	cycleECat();
	// Send a barrage of packets to set up the DC clock.
	int64_t stoptime = ec_DCtime + 200000000;
	// SOEM automatically updates ec_DCtime.
	while (ec_DCtime < stoptime) {
		cycleECat();
	}
	
	// We now have data.
	
	// Configure our medullas
	eCatConn->getMedullaManager()->start(ec_slave, ec_slavecount);
	
	done = false;
	return !done;
}

void ConnManager::loop() {
	RTT::os::TimeService::nsecs overshoot  = 0;
	RTT::os::TimeService::nsecs targetTime = RTT::os::TimeService::Instance()->getNSecs();
	RTT::os::TimeService::nsecs sleepTime  = 0;
	while (!done) {
		// This is used to compensate for timing overshoots when adjusting to match the DC clock.
		overshoot = RTT::os::TimeService::Instance()->getNSecs() - targetTime;
		
		int64_t eCatTime;
		{
			RTT::os::MutexLock lock(eCatLock);
			cycleECat();
			eCatTime = ec_DCtime;
			eCatConn->getMedullaManager()->processReceiveData();
		}
		eCatConn->getMedullaManager()->setTime(
			(eCatTime + CONTROLLER_LOOP_OFFSET_NS) -
			(eCatTime + CONTROLLER_LOOP_OFFSET_NS) % CONTROLLER_LOOP_OFFSET_NS);
		
		eCatConn->newStateCallback(eCatConn->getMedullaManager()->getRobotState());
	
		// The division here functions as an IIR filter on the DC time.
		RTT::os::TimeService::nsecs dcCorrection =
			-((eCatTime-overshoot+CONTROLLER_LOOP_PERIOD_NS/2) % CONTROLLER_LOOP_PERIOD_NS
			- CONTROLLER_LOOP_PERIOD_NS/2) / TIMING_FILTER_GAIN;
		
		RTT::os::TimeService::nsecs cur_time     = RTT::os::TimeService::Instance()->getNSecs();
		// Note: % is not actually modulo... hence the additional CONTROLLER_LOOP_PERIOD_NS
		sleepTime                                =
			(targetTime + dcCorrection - cur_time) % CONTROLLER_LOOP_PERIOD_NS + CONTROLLER_LOOP_PERIOD_NS;
		targetTime = sleepTime + cur_time;
		nanosleep(sleepTime);
	}
	
	// Sleep to make sure medullas make it to idle before stopping the DC.
	nanosleep(2*CONTROLLER_LOOP_PERIOD_NS);
}

void ConnManager::sendControllerOutput(atrias_msgs::controller_output controller_output) {
	RTT::os::MutexLock lock(eCatLock);
	eCatConn->getMedullaManager()->processTransmitData(controller_output);
	cycleECat();		
}

bool ConnManager::breakLoop() {
	done = true;
	return done;
}

}

}
