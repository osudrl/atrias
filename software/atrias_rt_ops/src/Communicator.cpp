#include "atrias_rt_ops/Communicator.h"

Communicator::Communicator(RTT::Seconds loop_period) :
              Activity(ORO_SCHED_RT, RTT::os::HighestPriority, loop_period) {
	return;
}

bool Communicator::init() {
	return false;
}

void Communicator::transmitHook() {
	return;
}

void Communicator::disable() {
	return;
}

void Communicator::enable() {
	return;
}

void Communicator::eStop() {
	return;
}

void Communicator::leaveEStop() {
	return;
}
