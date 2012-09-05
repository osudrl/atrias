#include "atrias_ecat_conn/ConnManager.h"

void sig_handler(int signum) {
	return;
}

namespace atrias {

namespace ecatConn {

ConnManager::ConnManager(ECatConn* ecat_conn) :
             RTT::os::Timer(1, ORO_SCHED_RT, 80) {
	eCatConn = ecat_conn;
	signal(SIGXCPU, sig_handler);
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

bool ConnManager::configure() {
	if (!ec_init("rteth0")) {
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

	targetTime = sleepTime + cur_time + filtered_overshoot;

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
	//log(RTT::Info) << "################" << RTT::endlog();
	if (isArmed(0))
		killTimer(0);
	
	// Sleep to make sure medullas make it to idle before stopping the DC.
	usleep(2*CONTROLLER_LOOP_PERIOD_NS/1000);
}

}

}
