#include <atrias_rt_ops/RTOps.h>

namespace atrias {

namespace rtOps {

RTOps::RTOps(std::string name) :
       RTT::TaskContext(name),
       timestampHandler(),
       rtHandler(),
       sendControllerOutput()
{
	this->provides("timestamps")
	    ->addOperation("getTimestamp", &RTOps::getTimestamp, this, RTT::ClientThread)
	    .doc("Get timestamp.");
	this->provides("timestamps")
	    ->addOperation("getROSHeader", &RTOps::getROSHeader, this, RTT::ClientThread);
	this->provides("rtOps")
	    ->addOperation("newStateCallback", &RTOps::newStateCallback, this, RTT::ClientThread);
	this->requires("connector")
	    ->addOperationCaller(sendControllerOutput);

	cmComms           = new CMComms(this);
	guiComms          = new GuiComms(this);
	controllerLoop    = new ControllerLoop(this);
	opsLogger         = new OpsLogger(this);
	stateMachine      = new StateMachine(this);
	robotStateHandler = new RobotStateHandler(this);
	safety            = new Safety(this);

	log(RTT::Info) << "[RTOps] constructed!" << RTT::endlog();
}

uint64_t RTOps::getTimestamp() {
	return timestampHandler.getTimestamp();
}

GuiComms* RTOps::getGuiComms() {
	return guiComms;
}

std_msgs::Header RTOps::getROSHeader() {
	return timestampHandler.getTimestampHeader();
}

void RTOps::newStateCallback(atrias_msgs::robot_state state) {
	// Order is important here -- the OpsLogger needs to have the last
	// cycle's robot state when it logs last cycle's data.
	opsLogger->beginCycle();
	robotStateHandler->setRobotState(state);
	
	controllerLoop->cycleLoop();
}

RobotStateHandler* RTOps::getRobotStateHandler() {
	return robotStateHandler;
}

TimestampHandler* RTOps::getTimestampHandler() {
	return &timestampHandler;
}

OpsLogger* RTOps::getOpsLogger() {
	return opsLogger;
}

ControllerLoop* RTOps::getControllerLoop() {
	return controllerLoop;
}

StateMachine* RTOps::getStateMachine() {
	return stateMachine;
}

Safety* RTOps::getSafety() {
	return safety;
}

void RTOps::sendEvent(atrias_msgs::rt_ops_event event) {
	opsLogger->sendEvent(event);
}

bool RTOps::configureHook() {
	rtHandler.beginRT();
	
	// Connect with the connector.
	RTT::TaskContext *peer = this->getPeer("atrias_connector");
	if (!peer) {
		log(RTT::Error) << "[RTOps] Failed to connect to a Connector!" << RTT::endlog();
		return false;
	}
	sendControllerOutput = peer->provides("connector")->getOperation("sendControllerOutput");
	
	return true;
}

bool RTOps::startHook() {
	// Start the main control loop.
	if (!controllerLoop->start()) {
		log(RTT::Error) << "[RTOps] Controller loop failed to start!" << RTT::endlog();
		return false;
	}
	log(RTT::Info) << "[RTOps] started." << RTT::endlog();
	return true;
}

void RTOps::updateHook() {
	cmComms->updateHook();
	return;
}

void RTOps::stopHook() {
	// Stop the control loop.
	if (!controllerLoop->stop())
		log(RTT::Error) << "[RTOps] Controller loop failed to stop! Continuing shutdown" << RTT::endlog();
	
	rtHandler.endRT();

	log(RTT::Info) << "[RTOps] stopped!" << RTT::endlog();
}

void RTOps::cleanupHook() {
	delete(cmComms);
	delete(guiComms);
	delete(controllerLoop);
	delete(opsLogger);
	delete(stateMachine);
	delete(robotStateHandler);
	delete(safety);
	log(RTT::Info) << "[RTOps] cleaned up!" << RTT::endlog();
}

ORO_CREATE_COMPONENT(RTOps)

}

}

// vim: noexpandtab
