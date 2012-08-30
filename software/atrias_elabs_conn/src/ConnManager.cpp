#include "atrias_elabs_conn/ConnManager.h"

namespace atrias {

namespace elabsConn {

ConnManager::ConnManager(ELabsConn* elabs_conn) :
             RTT::Activity(ETHERCAT_PRIO,
             ((double) CONTROLLER_LOOP_PERIOD_NS) / ((double) SECOND_IN_NANOSECONDS)) {
	eLabsConn = elabs_conn;
}

ConnManager::~ConnManager() {
	
}

void ConnManager::cyclic() {
	ecrt_master_application_time(master, RTT::os::TimeService::Instance()->getNSecs());
	ecrt_master_sync_reference_clock(master);
	ecrt_master_sync_slave_clocks(master);
	ecrt_domain_queue(domain);
	ecrt_master_send(master);
	timespec delay = {
		0,
		RECEIVE_WAIT_TIME_NS
	};
	clock_nanosleep(CLOCK_MONOTONIC, 0, &delay, NULL);
	ecrt_master_receive(master);
	ecrt_domain_process(domain);
}

bool ConnManager::configure() {
	log(RTT::Info) << "Beginning EtherCAT init" << endlog();
	CstructMstrAttach MstrAttach;
	MstrAttach.masterindex = 0;
	master = ecrt_request_master(MstrAttach.masterindex);
	if (!master) {
		log(RTT::Error) << __FILE__ << ": ecrt_request_master() FAILED!" << endlog();
		return false;
	}
	
	domain = ecrt_master_create_domain(master);
	if (!domain) {
		log(RTT::Error) << __FILE__ << ": ecrt_master_create_domain() FAILED!" << endlog();
		return false;
	}
	
	medullaManager = new MedullaManager(rtOps, master, domain);
	
	slavesManager->processTransmitData();
	
	log(RTT::Info) << "Slaves configuration complete, waiting for OP" << endlog();
	ec_master_state_t master_state;
	do {
		cyclic();
		ecrt_master_state(master, &master_state);
	} while (master_state.al_states != ELABS_OP_STATE);
	log(RTT::Info) << "All slaves are now in OP." << endlog();
	
	slavesManager->processReceiveData();
	
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
	
	targetTime         = RTT::os::TimeService::Instance()->getNSecs();
	filtered_overshoot = 0;
	done               = false;
	arm(0, 0.0);
	
	return !done;
}

void ConnManager::timeout(TimerId timer_id) {
	//log(RTT::Info) << "> 4" << RTT::endlog();
	filtered_overshoot += (RTT::os::TimeService::Instance()->getNSecs() - 
	                       targetTime - filtered_overshoot) / TIMING_FILTER_GAIN;
	
	//log(RTT::Info) << "< 1" << RTT::endlog();
	// Prevent undershooting.
	while (RTT::os::TimeService::Instance()->getNSecs() < targetTime);
	//log(RTT::Info) << "> 1" << RTT::endlog();
	
	// This is used to compensate for timing overshoots when adjusting to match the DC clock.
	RTT::os::TimeService::nsecs overshoot = RTT::os::TimeService::Instance()->getNSecs() - targetTime;

	//log(RTT::Info) << overshoot << RTT::endlog();

	int64_t eCatTime;
	{
		//log(RTT::Info) << "< 2" << RTT::endlog();
		RTT::os::MutexLock lock(eCatLock);
		//log(RTT::Info) << "> 2" << RTT::endlog();
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

	//log(RTT::Info) << dcCorrection << RTT::endlog();

	RTT::os::TimeService::nsecs cur_time     = RTT::os::TimeService::Instance()->getNSecs();
	// Note: % is not actually modulo... hence the additional CONTROLLER_LOOP_PERIOD_NS
	RTT::os::TimeService::nsecs sleepTime    =
		(targetTime + dcCorrection - filtered_overshoot - cur_time) % CONTROLLER_LOOP_PERIOD_NS + CONTROLLER_LOOP_PERIOD_NS;

	//log(RTT::Info) << sleepTime << RTT::endlog();

	targetTime = sleepTime + cur_time;
	{
		//log(RTT::Info) << "< 3" << RTT::endlog();
		RTT::os::MutexLock lock(timerLock);
		//log(RTT::Info) << "> 3" << RTT::endlog();
		if (!done) {
			//log(RTT::Info) << sleepTime << RTT::endlog();
			arm(timer_id, ((double) sleepTime) / ((double) SECOND_IN_NANOSECONDS));
		}
	}
	//log(RTT::Info) << "< 4" << RTT::endlog();
}

void ConnManager::sendControllerOutput(atrias_msgs::controller_output& controller_output) {
	RTT::os::MutexLock lock(eCatLock);
	eCatConn->getMedullaManager()->processTransmitData(controller_output);
	cycleECat();
}

void ConnManager::stop() {
	done = true;
	RTT::os::MutexLock lock(timerLock);
	log(RTT::Info) << "################" << RTT::endlog();
	if (isArmed(0))
		killTimer(0);
	
	// Sleep to make sure medullas make it to idle before stopping the DC.
	usleep(2*CONTROLLER_LOOP_PERIOD_NS/1000);
}

}

}
