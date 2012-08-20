#include "atrias_rt_ops/NoopConnector.h"

NoopConnector::NoopConnector(RTOps* rt_ops) :
                 Communicator(((double) CONTROLLER_LOOP_PERIOD_NS) / 1000000000.0) {
	rtOps = rt_ops;
}

bool NoopConnector::init() {
	return true;
}

void NoopConnector::step() {
	{
		RTT::os::MutexLock lock(rtOps->robotStateLock);
		rtOps->rtOpsCycle.header = rtOps->getRosHeader();
	}
	rtOps->newStateCallback();
}
