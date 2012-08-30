#include "atrias_elabs_conn/ELabsConn.h"

namespace atrias {

namespace elabsConn {

ELabsConn::ELabsConn(std::string name) :
          RTT::TaskContext(name),
          newStateCallback("newStateCallback") {
	this->provides("connector")
	    ->addOperation("sendControllerOutput", &ELabsConn::sendControllerOutput, this, RTT::ClientThread);
	this->requires("atrias_rt")
	    ->addOperationCaller(newStateCallback);
	this->requires("atrias_rt")
	    ->addOperationCaller(sendEvent);
	
	connManager = new ConnManager(this);
	
	log(RTT::Info) << "[ELabsConn] constructed." << RTT::endlog();
}

bool ELabsConn::configureHook() {
	RTT::TaskContext *peer = this->getPeer("atrias_rt");
	if (!peer) {
		log(RTT::Error) << "[ELabsConn] Failed to connect to RTOps!" << RTT::endlog();
		return false;
	}
	newStateCallback = peer->provides("rtOps")->getOperation("newStateCallback");
	sendEvent        = peer->provides("rtOps")->getOperation("sendEvent");
	
	if (!connManager->configure()) {
		log(RTT::Error) << "[ELabsConn] ConnManager failed to configure!" << RTT::endlog();
		return false;
	}
	
	log(RTT::Info) << "[ELabsConn] configured." << RTT::endlog();
	return true;
}

bool ELabsConn::startHook() {
	if (!connManager->initialize()) {
		log(RTT::Error) << "[ELabsConn] ConnManager failed to start!" << RTT::endlog();
		return false;
	}
	
	log(RTT::Info) << "[ELabsConn] started." << RTT::endlog();
	
	return true;
}

void ELabsConn::stopHook() {
	connManager->stop();
	
	log(RTT::Info) << "[ELabsConn] stopped." << RTT::endlog();
}

void ELabsConn::sendControllerOutput(atrias_msgs::controller_output controller_output) {
	connManager->sendControllerOutput(controller_output);
	return;
}

ORO_CREATE_COMPONENT(ELabsConn)

}

}
