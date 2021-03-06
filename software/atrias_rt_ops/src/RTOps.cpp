#include <atrias_rt_ops/RTOps.h>

namespace atrias {

namespace rtOps {

RTOps::RTOps(std::string name) :
       RTT::TaskContext(name),
       cManagerDataIn("controller_manager_data_in"),
       logCyclicOut("rt_ops_log_out"),
       guiCyclicOut("rt_ops_gui_out"),
       eventOut("rt_ops_event_out"),
       timestampHandler(),
       opsLogger(&logCyclicOut, &guiCyclicOut, &eventOut),
       rtHandler(),
       runController("runController"),
       sendControllerOutput()
{
	this->provides("timestamps")
	    ->addOperation("getTimestamp", &RTOps::getTimestamp, this, RTT::ClientThread)
	    .doc("Get timestamp.");
	this->provides("timestamps")
	    ->addOperation("getROSHeader", &RTOps::getROSHeader, this, RTT::ClientThread);
	this->requires("atc")
	    ->addOperationCaller(runController);
	this->provides("rtOps")
	    ->addOperation("newStateCallback", &RTOps::newStateCallback, this, RTT::ClientThread);
	this->requires("connector")
	    ->addOperationCaller(sendControllerOutput);
	this->provides("rtOps")
	    ->addOperation("sendEvent", &RTOps::sendEvent, this, RTT::ClientThread);
	    
	addEventPort(cManagerDataIn);
	addPort(logCyclicOut);
	addPort(guiCyclicOut);
	addPort(eventOut);

	eStopDiags        = new EStopDiags(this);
	controllerLoop    = new ControllerLoop(this);
	stateMachine      = new StateMachine(this);
	robotStateHandler = new RobotStateHandler(this);
	safety            = new Safety(this);

	log(RTT::Info) << "[RTOps] constructed!" << RTT::endlog();
}

void RTOps::connectToController() {
	RTT::TaskContext *peer = this->getPeer("controller");
	if (peer)
		this->connectServices(peer);
	
	// Connection is done automatically by Orocos... just check to see if it succeeded
	if (this->requires("atc")->ready()) {
		log(RTT::Info) << "Service ATC ready" << RTT::endlog();
	} else {
		log(RTT::Warning) << "Service ATC Not ready!" << RTT::endlog();
	}

	if (this->runController.ready())
		log(RTT::Info) << "runController ready." << RTT::endlog();
	else
		log(RTT::Warning) << "runController not ready!" << RTT::endlog();
}

void RTOps::disconnectController() {
	runController.disconnect();
}

uint64_t RTOps::getTimestamp() {
	return timestampHandler.getTimestamp();
}

std_msgs::Header RTOps::getROSHeader() {
	return timestampHandler.getTimestampHeader();
}

void RTOps::newStateCallback(atrias_msgs::robot_state state) {
	opsLogger.beginCycle();
	robotStateHandler->setRobotState(state);
	
	controllerLoop->cycleLoop();
	
	opsLogger.logRobotState(state);
}

EStopDiags* RTOps::getEStopDiags() const {
	return eStopDiags;
}

RobotStateHandler* RTOps::getRobotStateHandler() {
	return robotStateHandler;
}

TimestampHandler* RTOps::getTimestampHandler() {
	return &timestampHandler;
}

OpsLogger* RTOps::getOpsLogger() {
	return &opsLogger;
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

void RTOps::sendEvent(RtOpsEvent event, RtOpsEventMetadata_t metadata) {
	opsLogger.sendEvent(event, metadata);
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
	if (RTT::NewData == cManagerDataIn.read(cmIn) && stateMachine)
		stateMachine->newCMState((RtOpsState) cmIn);
	
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
	log(RTT::Info) << "[RTOps] cleaned up!" << RTT::endlog();
}

ORO_CREATE_COMPONENT(RTOps)

}

}

// vim: noexpandtab
