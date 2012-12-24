#include "atrias_rt_ops/OpsLogger.h"

namespace atrias {

namespace rtOps {

OpsLogger::OpsLogger(RTOps *rt_ops) :
		logCyclicOut("rt_ops_log_out"),
		guiCyclicOut("rt_ops_gui_out"),
		eventOut("rt_ops_event_out"),
		guiPublishTimer(50)
{
	rt_ops->provides("rtOps")
	      ->addOperation("sendEvent", &RTOps::sendEvent, rt_ops, RTT::ClientThread);

	rt_ops->addPort(logCyclicOut);
	rt_ops->addPort(guiCyclicOut);
	rt_ops->addPort(eventOut);
}

void OpsLogger::beginCycle() {
	RTT::os::TimeService::nsecs startTime = RTT::os::TimeService::Instance()->getNSecs();
	
	// Send our 1 kHz log stream.
	logCyclicOut.write(rtOpsCycle);
	if (guiPublishTimer.readyToSend()) {
		// Time to send on the 50 Hz port.
		guiCyclicOut.write(rtOpsCycle);
	}
	
	rtOpsCycle.startTime = startTime;
}

void OpsLogger::logControllerOutput(atrias_msgs::controller_output& output) {
	rtOpsCycle.controllerOutput = output;
}

void OpsLogger::logClampedControllerOutput(
	atrias_msgs::controller_output& clamped_output) {
	rtOpsCycle.commandedOutput = clamped_output;
}

void OpsLogger::endCycle() {
	rtOpsCycle.endTime = RTT::os::TimeService::Instance()->getNSecs();
}

void OpsLogger::logRobotState(atrias_msgs::robot_state& state) {
	rtOpsCycle.robotState = state;
	rtOpsCycle.header     = state.header;
}

void OpsLogger::sendEvent(atrias_msgs::rt_ops_event &event) {
	eventOut.write(event);
}

}

}

// vim: noexpandtab
