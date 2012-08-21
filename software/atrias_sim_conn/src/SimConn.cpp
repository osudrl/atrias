#include "atrias_sim_conn/SimConn.h"

namespace atrias {

namespace simConn {

SimConn::SimConn(std::string name) :
         RTT::TaskContext(name),
         newStateCallback("newStateCallback") {
	this->provides("connector")
	    ->addOperation("sendControllerOutput", &SimConn::sendControllerOutput, this, RTT::ClientThread);
	this->requires("atrias_rt")
	    ->addOperationCaller(newStateCallback);
}

bool SimConn::configureHook() {
	RTT::TaskContext *peer = this->getPeer("atrias_rt");
	if (!peer) {
		log(RTT::Error) << "[SimConn] Failed to connect to RTOps!" << RTT::endlog();
		return false;
	}
	newStateCallback = peer->provides("rtOps")->getOperation("newStateCallback");
	log(RTT::Info) << "[SimConn] configured!" << RTT::endlog();
	return true;
}

void SimConn::sendControllerOutput(atrias_msgs::controller_output controller_output) {
	return;
}

ORO_CREATE_COMPONENT(SimConn)

}

}
