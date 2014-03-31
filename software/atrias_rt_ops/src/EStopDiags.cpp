// The include file for this class
#include "atrias_rt_ops/EStopDiags.hpp"

// The namespaces in which this class resides
namespace atrias {
namespace rtOps {

EStopDiags::EStopDiags(RTOps *rt_ops) : RTT::Service("EStopDiags", rt_ops) {
	// Diagnostic information for the user
	log(RTT::Debug) << "[RTOps] Initializing EStopDiags" << RTT::endlog();

	// Register our backend-calling operations
	log(RTT::Debug) << "[RTOps] Initializing EStop-printing Operation" << RTT::endlog();
	rt_ops->addOperation("printEStop", &EStopDiags::printEStopBackend, this, RTT::OwnThread)
		.doc("Print an EStop in an asynchronous manner.");

	// Configure the operation caller for the printing operation
	log(RTT::Debug) << "[RTOps] Initializing EStop-printing Operation" << RTT::endlog();
	this->printEStopCaller = rt_ops->getOperation("printEStop");
}

void EStopDiags::printEStop(RtOpsEvent &event) {
	// Simply send it asynchronously; it should be able to deal with it relatively quickly
	this->printEStopCaller.send(event);
}

void EStopDiags::printEStopBackend(RtOpsEvent event) {
	// Basic header
	log(RTT::Info) << "EStop occurred" << RTT::endlog();

	// Switch based on the event in question
	switch (event) {
		case RtOpsEvent::INVALID_CM_COMMAND:
			log(RTT::Info) << "EStop reason: Invalid Controller Manager Command. "
			               << "This should never happen!" << RTT::endlog();
			break;

		case RtOpsEvent::INVALID_RT_OPS_STATE:
			log(RTT::Info) << "EStop reason: Invalid RT Ops State. "
			               << "This should never happen!" << RTT::endlog();
			break;

		case RtOpsEvent::CM_COMMAND_ESTOP:
			log(RTT::Info) << "EStop reason: Controller Manager commanded an EStop. "
			               << "This is usually just a forwarded EStop command from the GUI."
			               << RTT::endlog();
			break;

		case RtOpsEvent::CONTROLLER_ESTOP:
			log(RTT::Info) << "EStop reason: Controller commanded an EStop. "
			               << RTT::endlog();
			break;

		case RtOpsEvent::MEDULLA_ESTOP:
			log(RTT::Info) << "EStop reason: One or more Medullas went into EStop automatically. "
			               << "Look at the Medulla error flags to identify the cause. "
			               << "Note: As a kludge, this is currently sent when the safeties detect "
			               << "a failing halt."
			               << RTT::endlog();
			break;

		case RtOpsEvent::SAFETY:
			log(RTT::Info) << "EStop reason: Safeties (hard stop collision avoidance) triggered. "
			               << RTT::endlog();
			break;

		default:
			log(RTT::Info) << "EStop reason unknown! Someone called eStop() without specifying "
			               << "a valid reason. Reason given (integer): "
			               << (int) event << RTT::endlog();
			break;
	}
}

// End namespaces
}
}

// This file uses tab-based indentation
// vim: noexpandtab
