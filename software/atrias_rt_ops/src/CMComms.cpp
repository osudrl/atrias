#include "atrias_rt_ops/CMComms.h"

namespace atrias {

namespace rtOps {

CMComms::CMComms(RTOps *rt_ops) :
	cmInPort("controller_manager_data_in")
{
	rtOps = rt_ops;
	rtOps->addEventPort(cmInPort);
}

bool CMComms::updateHook() {
	// Check for a new command on the input port.
	if (cmInPort.read(cmIn)) {
		// This will be used in the acknowledgement event
		AckCmEventMetadata metadata;

		switch (cmIn) {
			case controllerManager::ControllerManagerCommand::UNLOAD_CONTROLLER:
				// We've been asked to unload the controller.
				rtOps->getControllerLoop()->setControllerUnloaded();
				// This never fails, so respond with the same message every time.
				metadata = AckCmEventMetadata::CONTROLLER_UNLOADED;
				break;
			case controllerManager::ControllerManagerCommand::CONTROLLER_LOADED:
				// This can fail, however, so send back appropriate metadata for the event.
				metadata = rtOps->getControllerLoop()->loadController();
				break;
			default:
				// Ack! This command isn't recognized...
				metadata = AckCmEventMetadata::UNKNOWN_CMD;
				break;
		}
		// Send the acknowledgement back to the CM
		rtOps->sendEvent(buildEventMetadata(RtOpsEvent::ACK_CM, metadata));

		// Tell RT Ops we've handled the updateHook.
		return true;
	}

	// If we get here, then there hasn't been a new command -- let RTOps know.
	return false;
}

}

}
