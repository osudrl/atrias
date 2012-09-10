#include "atrias_ecat_conn/ECatConn.h"

namespace atrias {

namespace ecatConn {

ECatConn::ECatConn(std::string name) :
          RTT::TaskContext(name),
          newStateCallback("newStateCallback") {
	this->provides("connector")
	    ->addOperation("sendControllerOutput", &ECatConn::sendControllerOutput, this, RTT::ClientThread);
	this->provides("disableSafeties")
	    ->addOperation("disableSafeties", &ECatConn::disableSafeties, this, RTT::ClientThread);
	this->requires("atrias_rt")
	    ->addOperationCaller(newStateCallback);
	this->requires("atrias_rt")
	    ->addOperationCaller(sendEvent);
	
	connManager = new ConnManager(this);
	
	log(RTT::Info) << "[ECatConn] constructed." << RTT::endlog();
}

bool ECatConn::configureHook() {
	RTT::TaskContext *peer = this->getPeer("atrias_rt");
	if (!peer) {
		log(RTT::Error) << "[ECatConn] Failed to connect to RTOps!" << RTT::endlog();
		return false;
	}
	newStateCallback = peer->provides("rtOps")->getOperation("newStateCallback");
	sendEvent        = peer->provides("rtOps")->getOperation("sendEvent");
	
	if (!connManager->configure()) {
		log(RTT::Error) << "[ECatConn] ConnManager failed to configure!" << RTT::endlog();
		return false;
	}
	
	log(RTT::Info) << "[ECatConn] configured." << RTT::endlog();
	return true;
}

bool ECatConn::startHook() {
	if (!connManager->start()) {
		log(RTT::Error) << "[ECatConn] ConnManager failed to start!" << RTT::endlog();
		return false;
	}
	
	log(RTT::Info) << "[ECatConn] started." << RTT::endlog();
	
	return true;
}

void ECatConn::stopHook() {
	connManager->stop();
	
	log(RTT::Info) << "[ECatConn] stopped." << RTT::endlog();
}

void ECatConn::sendControllerOutput(atrias_msgs::controller_output controller_output) {
	connManager->sendControllerOutput(controller_output);
	return;
}

MedullaManager* ECatConn::getMedullaManager() {
	return &medullaManager;
}

void ECatConn::disableSafeties() {
	medullaManager.setRobotConfiguration(rtOps::RobotConfiguration::DISABLE);
}

ORO_CREATE_COMPONENT(ECatConn)

}

}
