#include "atrias_rt_ops/ControllerLoop.h"

namespace atrias {

namespace rtOps {

ControllerLoop::ControllerLoop(RTOps* rt_ops) :
                RTT::Activity(70),
                signal(0),
				runController() {
	rtOps            = rt_ops;
	controllerLoaded = false;
	rtOps->requires("atc")->
		addOperationCaller(runController);
}

AckCmEventMetadata ControllerLoop::loadController() {
	RTT::TaskContext *controller = rtOps->getPeer("controller");
	if (!controller)
		return AckCmEventMetadata::NO_PEER;
	
	runController = controller->provides("atc")->getOperation("runController");
	if (!runController.ready())
		return AckCmEventMetadata::NO_OPERATION;

	controllerLoaded = true;
	return AckCmEventMetadata::CONTROLLER_LOADED;
}

void ControllerLoop::setControllerUnloaded() {
	if (!controllerLoaded) {
		// The controller's already been unloaded.
		return;
	}

	// The mutex prevents concurrency issues here (see loop() ).
	controllerLoaded = false;
	RTT::os::MutexLock lock(controllerLock);
}

atrias_msgs::controller_output
	ControllerLoop::clampControllerOutput(
	atrias_msgs::controller_output controller_output) {
	
	controller_output.lLeg.motorCurrentA =
		CLAMP(controller_output.lLeg.motorCurrentA, MIN_MTR_TRQ_CMD, MAX_MTR_TRQ_CMD);
	controller_output.lLeg.motorCurrentB =
		CLAMP(controller_output.lLeg.motorCurrentB, MIN_MTR_TRQ_CMD, MAX_MTR_TRQ_CMD);
	controller_output.lLeg.motorCurrentHip =
		CLAMP(controller_output.lLeg.motorCurrentHip, MIN_HIP_MTR_TRQ_CMD, MAX_HIP_MTR_TRQ_CMD);
	
	controller_output.rLeg.motorCurrentA =
		CLAMP(controller_output.rLeg.motorCurrentA, MIN_MTR_TRQ_CMD, MAX_MTR_TRQ_CMD);
	controller_output.rLeg.motorCurrentB =
		CLAMP(controller_output.rLeg.motorCurrentB, MIN_MTR_TRQ_CMD, MAX_MTR_TRQ_CMD);
	controller_output.rLeg.motorCurrentHip =
		CLAMP(controller_output.rLeg.motorCurrentHip, MIN_HIP_MTR_TRQ_CMD, MAX_HIP_MTR_TRQ_CMD);
	
	return controller_output;
}

void ControllerLoop::zeroTorques(atrias_msgs::controller_output &controller_output) {
	controller_output.lLeg.motorCurrentA   = 0.0;
	controller_output.lLeg.motorCurrentB   = 0.0;
	controller_output.lLeg.motorCurrentHip = 0.0;
	controller_output.rLeg.motorCurrentA   = 0.0;
	controller_output.rLeg.motorCurrentB   = 0.0;
	controller_output.rLeg.motorCurrentHip = 0.0;
}

void ControllerLoop::loop() {
	while (!done) {
		rtOps->getStateMachine()->run();
		atrias_msgs::robot_state robotState = rtOps->getRobotStateHandler()->getRobotState();
		rtOps->getTimestampHandler()->setTimestamp(robotState.header);
		
		atrias_msgs::controller_output controllerOutput;
		zeroTorques(controllerOutput);
		
		{
			RTT::os::MutexLock lock(controllerLock);
			if (controllerLoaded) {
				controllerOutput = runController(robotState);
			}
		}
		if ((RtOpsState) robotState.rtOpsState == RtOpsState::STOP) {
			// Run the stop controller here.
		}
		
		rtOps->getOpsLogger()->logControllerOutput(controllerOutput);

		controllerOutput.command = rtOps->getStateMachine()->calcMedullaCmd((RtOpsState) controllerOutput.command);
		controllerOutput = clampControllerOutput(controllerOutput);

		rtOps->getOpsLogger()->logClampedControllerOutput(controllerOutput);
		
		if (controllerOutput.command != medulla_state_run) {
			// Robot should be disabled, so zero current commands.
			zeroTorques(controllerOutput);
		}
		
		rtOps->sendControllerOutput(controllerOutput);
		rtOps->getOpsLogger()->endCycle();
		
		signal.wait();
	}
}

void ControllerLoop::cycleLoop() {
	RTT::os::MutexLock lock(signalLock);
	if (!signal.value())
		signal.signal();
}

bool ControllerLoop::breakLoop() {
	done = true;
	cycleLoop();
	return done;
}

bool ControllerLoop::initialize() {
	done = false;
	return !done;
}

}

}
