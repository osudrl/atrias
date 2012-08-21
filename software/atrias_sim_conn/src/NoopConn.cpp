#include "atrias_noop_conn/NoopConn.h"

namespace atrias {

namespace noopConn {

NoopConn::NoopConn(std::string name) :
          RTT::TaskContext(name),
          newStateCallback("newStateCallback") {
	this->provides("connector")
	    ->addOperation("sendControllerOutput", &NoopConn::sendControllerOutput, this, RTT::ClientThread);
	this->requires("atrias_rt")
	    ->addOperationCaller(newStateCallback);
}

bool NoopConn::configureHook() {
	RTT::TaskContext *peer = this->getPeer("atrias_rt");
	if (!peer) {
		log(RTT::Error) << "[NoopConn] Failed to connect to RTOps!" << RTT::endlog();
		return false;
	}
	newStateCallback = peer->provides("rtOps")->getOperation("newStateCallback");
	log(RTT::Info) << "[NoopConn] configured!" << RTT::endlog();
	return true;
}

void NoopConn::sendControllerOutput(atrias_msgs::controller_output controller_output) {
	return;
}

void NoopConn::updateHook() {
	RTT::os::TimeService::nsecs timestamp = RTT::os::TimeService::Instance()->getNSecs();
	robotState.header.stamp.sec  = timestamp / SECOND_IN_NANOSECONDS;
	robotState.header.stamp.nsec = timestamp % SECOND_IN_NANOSECONDS;
	newStateCallback(robotState);
}

ORO_CREATE_COMPONENT(NoopConn)

}

}
