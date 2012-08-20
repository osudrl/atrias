#include "atrias_rt_ops/ECatComm.h"

ECatComm::ECatComm(RTOps* rt_ops) :
          Communicator(0) {
	rtOps            = rt_ops;
}

void ECatComm::transmitHook() {
	if (slavesManager)
		slavesManager->processTransmitData();

	RTT::os::MutexLock lock(eCatLock);
	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUT_US);
	/* todo: <log transmit time here> */
}

void ECatComm::nanosleep(RTT::os::TimeService::nsecs sleeptime) {
	timespec delay = {
		0,
		(long) sleeptime
	};
	clock_nanosleep(CLOCK_MONOTONIC, 0, &delay, NULL);
}

void ECatComm::loop() {
	RTT::os::TimeService::nsecs overshoot  = 0;
	RTT::os::TimeService::nsecs targetTime = RTT::os::TimeService::Instance()->getNSecs();
	RTT::os::TimeService::nsecs sleepTime  = 0;
	while (!done) {
		{
			// Log data here.
			
			// This is used to compensate for timing overshoots when adjusting to match the DC clock.
			overshoot = RTT::os::TimeService::Instance()->getNSecs() - targetTime;
			
			int64_t eCatTime;
			{
                RTT::os::MutexLock lock(eCatLock);
                
                ec_send_processdata();
                
                ec_receive_processdata(EC_TIMEOUT_US);
		
				// Used later.
				eCatTime               = ec_DCtime;
			}
			
			rtOps->sendRobotState();
			
			// Process our newly-received data.
			{
                RTT::os::MutexLock lock(rtOps->robotStateLock);

				RTT::os::TimeService::nsecs delta =
					eCatTime - (eCatTime % CONTROLLER_LOOP_PERIOD_NS) - rtOps->currentTimestamp;
				if (delta > /*3 * */CONTROLLER_LOOP_PERIOD_NS)		
					printf("%llu\n", delta);
				rtOps->currentTimestamp += delta;
                slavesManager->processReceiveData();
			}
		
			// Run the controllers.
            rtOps->newStateCallback();
		
			// The division here functions as an IIR filter on the DC time.
			RTT::os::TimeService::nsecs dcCorrection =
				-((eCatTime-overshoot+CONTROLLER_LOOP_PERIOD_NS/2) % CONTROLLER_LOOP_PERIOD_NS
				- CONTROLLER_LOOP_PERIOD_NS/2) / TIMING_FILTER_GAIN;
			
			RTT::os::TimeService::nsecs cur_time     = RTT::os::TimeService::Instance()->getNSecs();
			// Note: % is not actually modulo... hence the additional CONTROLLER_LOOP_PERIOD_NS
			sleepTime                                =
				(targetTime + dcCorrection - cur_time) % CONTROLLER_LOOP_PERIOD_NS + CONTROLLER_LOOP_PERIOD_NS;
			targetTime = sleepTime + cur_time;
		}
		nanosleep(sleepTime);
	}
	disable();
	transmitHook();
	
	// Sleep to make sure medullas make it to idle before stopping the DC.
	nanosleep(2*CONTROLLER_LOOP_PERIOD_NS);
}

bool ECatComm::breakLoop() {
	done = true;
	return true;
}

void ECatComm::finalize() {
	delete(slavesManager);
	ec_slave[0].state = EC_STATE_INIT;
	ec_writestate(0);
	ec_close();
}

bool ECatComm::init() {
	if (!ec_init("eth0")) {
		printf("%s: ec_init() failed!\n", __FILE__);
		return false;
	}
	
	ec_config_init(FALSE);
	
	printf("%d slaves identified\n", ec_slavecount);
	if (ec_slavecount < 1) {
		printf("%s: failed to identify any slaves; skipping ECat init.\n", __FILE__);
		return false;
	}
	
	ec_configdc();
	
	ec_config_map(IOmap);
	
	// Wait for SAFE-OP
	ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
	
	return true;
}

bool ECatComm::initialize() {
	this->setCpuAffinity(rtOps->getActivity()->getCpuAffinity());	

	// Send them to OP
	ec_slave[0].state = EC_STATE_OPERATIONAL;
	
	/* simple_test.c says sending and processing a frame cycle can help here.
	 * Doesn't seem necessary, though (see ebox.c (an SOEM example) -- maybe
	 * it's for slaves w/ direct I/O?
	 */
	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUT_US);
	
	ec_writestate(0);
	ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);
	
	// We are now in OP.
	// Configure the distributed clocks for each slave.
	for (int i = 1; i <= ec_slavecount; i++) {
		if (ec_slave[i].eep_man == MEDULLA_VENDOR_ID)
			ec_dcsync0(i, true, 1000000, CONTROLLER_LOOP_PERIOD_NS - CONTROLLER_LOOP_OFFSET_NS);
	}
	
	// Update ec_DCtime so we can calculate stoptime below.
	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUT_US);
	// We need to send a barrage of packets in order to set up the DC clock
	// and actually start transferring data. This takes 100 ms.
	int64_t stoptime = ec_DCtime + 200000000;
	while (ec_DCtime < stoptime) {
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUT_US);
	}
	
	// We now have data.
	
	/* SOEM uses 1-indexing... slave 0 kindof refers to the entire system at
	 * once... in a way (see setting OP and SAFE-OP above).
	 */
	if (ec_slave[1].eep_man == MEDULLA_VENDOR_ID) {
		slavesManager = new MedullaManager(rtOps, ec_slave, ec_slavecount);
	}
	
	// Make sure our data is sane.
	transmitHook();
	
	done = false;
	
	return true;
}

void ECatComm::disable() {
	if(slavesManager)
		slavesManager->setEnabled(false);
}

void ECatComm::enable() {
	if (slavesManager)
		slavesManager->setEnabled(true);
}

void ECatComm::eStop() {
	printf("ESTOP!!!\n");
	if (slavesManager)
		slavesManager->eStop();
}

void ECatComm::leaveEStop() {
	if (slavesManager)
		slavesManager->leaveEStop();
}
