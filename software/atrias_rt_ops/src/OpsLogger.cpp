#include "atrias_rt_ops/OpsLogger.h"

namespace atrias {

namespace rtOps {

OpsLogger::OpsLogger(RTT::OutputPort<atrias_msgs::log_data>     *log_cyclic_out,
                     RTT::OutputPort<atrias_msgs::rt_ops_cycle> *gui_cyclic_out,
                     RTT::OutputPort<atrias_msgs::rt_ops_event> *event_out) :
                     guiPublishTimer(50) {
	logCyclicOut = log_cyclic_out;
	guiCyclicOut = gui_cyclic_out;
	eventOut     = event_out;
}

void OpsLogger::beginCycle() {
	RTT::os::TimeService::nsecs startTime = RTT::os::TimeService::Instance()->getNSecs();
	
	if (guiPublishTimer.readyToSend()) {
		// Time to send on the 50 Hz port.
		guiCyclicOut->write(rtOpsCycle);
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
	atrias_msgs::log_data log_data;
	packLogData(log_data);
	logCyclicOut->write(log_data);
	rtOpsCycle.header     = state.header;
	rtOpsCycle.robotState = state;
}

void OpsLogger::sendEvent(RtOpsEvent event, RtOpsEventMetadata_t metadata) {
	atrias_msgs::rt_ops_event event_msg;
	event_msg.event    = (RtOpsEvent_t) event;
	event_msg.metadata = metadata;
	eventOut->write(event_msg);
}

void OpsLogger::packLogData(atrias_msgs::log_data &ld) {
	atrias_msgs::robot_state &rs = this->rtOpsCycle.robotState;

	ld.header           = rs.header;

	ld.currentPositive  = rs.currentPositive;
	ld.currentNegative  = rs.currentNegative;

	ld.lAClampedCmd     = rtOpsCycle.commandedOutput.lLeg.motorCurrentA;
	ld.lBClampedCmd     = rtOpsCycle.commandedOutput.lLeg.motorCurrentB;
	ld.lHipClampedCmd   = rtOpsCycle.commandedOutput.lLeg.motorCurrentHip;
	ld.rAClampedCmd     = rtOpsCycle.commandedOutput.rLeg.motorCurrentA;
	ld.rBClampedCmd     = rtOpsCycle.commandedOutput.rLeg.motorCurrentB;
	ld.rHipClampedCmd   = rtOpsCycle.commandedOutput.rLeg.motorCurrentHip;

	ld.lKneeForce       = rs.lLeg.kneeForce;
	ld.rKneeForce       = rs.rLeg.kneeForce;

	ld.lALegAngle       = rs.lLeg.halfA.legAngle;
	ld.lBLegAngle       = rs.lLeg.halfB.legAngle;
	ld.rALegAngle       = rs.rLeg.halfA.legAngle;
	ld.rBLegAngle       = rs.rLeg.halfB.legAngle;

	ld.lALegVelocity    = rs.lLeg.halfA.legVelocity;
	ld.lBLegVelocity    = rs.lLeg.halfB.legVelocity;
	ld.rALegVelocity    = rs.rLeg.halfA.legVelocity;
	ld.rBLegVelocity    = rs.rLeg.halfB.legVelocity;

	ld.lAMotorAngle     = rs.lLeg.halfA.motorAngle;
	ld.lBMotorAngle     = rs.lLeg.halfB.motorAngle;
	ld.rAMotorAngle     = rs.rLeg.halfA.motorAngle;
	ld.rBMotorAngle     = rs.rLeg.halfB.motorAngle;

	ld.lAMotorVelocity  = rs.lLeg.halfA.motorVelocity;
	ld.lBMotorVelocity  = rs.lLeg.halfB.motorVelocity;
	ld.rAMotorVelocity  = rs.rLeg.halfA.motorVelocity;
	ld.rBMotorVelocity  = rs.rLeg.halfB.motorVelocity;

	ld.lARotorAngle     = rs.lLeg.halfA.rotorAngle;
	ld.lBRotorAngle     = rs.lLeg.halfB.rotorAngle;
	ld.rARotorAngle     = rs.rLeg.halfA.rotorAngle;
	ld.rBRotorAngle     = rs.rLeg.halfB.rotorAngle;

	ld.lARotorVelocity  = rs.lLeg.halfA.rotorVelocity;
	ld.lBRotorVelocity  = rs.lLeg.halfB.rotorVelocity;
	ld.rARotorVelocity  = rs.rLeg.halfA.rotorVelocity;
	ld.rBRotorVelocity  = rs.rLeg.halfB.rotorVelocity;

	ld.lLegBodyAngle    = rs.lLeg.hip.legBodyAngle;
	ld.rLegBodyAngle    = rs.rLeg.hip.legBodyAngle;

	ld.lLegBodyVelocity = rs.lLeg.hip.legBodyVelocity;
	ld.rLegBodyVelocity = rs.lLeg.hip.legBodyVelocity;

	ld.xPosition        = rs.position.xPosition;
	ld.xVelocity        = rs.position.xVelocity;
	ld.zPosition        = rs.position.zPosition;
	ld.zVelocity        = rs.position.zVelocity;

	ld.lToeSwitch       = rs.lLeg.toeSwitch;
	ld.rToeSwitch       = rs.rLeg.toeSwitch;

	ld.rtOpsState       = rs.rtOpsState;

	ld.controllerTime   = rs.timing.controllerTime;

	ld.lAMotorTherm0    = rs.lLeg.halfA.motorTherms[0];
	ld.lAMotorTherm1    = rs.lLeg.halfA.motorTherms[1];
	ld.lAMotorTherm2    = rs.lLeg.halfA.motorTherms[2];
	ld.lAMotorTherm3    = rs.lLeg.halfA.motorTherms[3];
	ld.lAMotorTherm4    = rs.lLeg.halfA.motorTherms[4];
	ld.lAMotorTherm5    = rs.lLeg.halfA.motorTherms[5];
	ld.lBMotorTherm0    = rs.lLeg.halfB.motorTherms[0];
	ld.lBMotorTherm1    = rs.lLeg.halfB.motorTherms[1];
	ld.lBMotorTherm2    = rs.lLeg.halfB.motorTherms[2];
	ld.lBMotorTherm3    = rs.lLeg.halfB.motorTherms[3];
	ld.lBMotorTherm4    = rs.lLeg.halfB.motorTherms[4];
	ld.lBMotorTherm5    = rs.lLeg.halfB.motorTherms[5];
	ld.rAMotorTherm0    = rs.rLeg.halfA.motorTherms[0];
	ld.rAMotorTherm1    = rs.rLeg.halfA.motorTherms[1];
	ld.rAMotorTherm2    = rs.rLeg.halfA.motorTherms[2];
	ld.rAMotorTherm3    = rs.rLeg.halfA.motorTherms[3];
	ld.rAMotorTherm4    = rs.rLeg.halfA.motorTherms[4];
	ld.rAMotorTherm5    = rs.rLeg.halfA.motorTherms[5];
	ld.rBMotorTherm0    = rs.rLeg.halfB.motorTherms[0];
	ld.rBMotorTherm1    = rs.rLeg.halfB.motorTherms[1];
	ld.rBMotorTherm2    = rs.rLeg.halfB.motorTherms[2];
	ld.rBMotorTherm3    = rs.rLeg.halfB.motorTherms[3];
	ld.rBMotorTherm4    = rs.rLeg.halfB.motorTherms[4];
	ld.rBMotorTherm5    = rs.rLeg.halfB.motorTherms[5];

	ld.lAMotorVoltage   = rs.lLeg.halfA.motorVoltage;
	ld.lBMotorVoltage   = rs.lLeg.halfB.motorVoltage;
	ld.lHipMotorVoltage = rs.lLeg.hip.motorVoltage;
	ld.rAMotorVoltage   = rs.rLeg.halfA.motorVoltage;
	ld.rBMotorVoltage   = rs.rLeg.halfB.motorVoltage;
	ld.rHipMotorVoltage = rs.rLeg.hip.motorVoltage;

	ld.lAMotorCurrent   = rs.lLeg.halfA.motorCurrent;
	ld.lBMotorCurrent   = rs.lLeg.halfB.motorCurrent;
	ld.rAMotorCurrent   = rs.rLeg.halfA.motorCurrent;
	ld.rBMotorCurrent   = rs.rLeg.halfB.motorCurrent;
}

}

}

// vim: noexpandtab
