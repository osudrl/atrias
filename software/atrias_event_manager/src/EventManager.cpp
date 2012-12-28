#include "atrias_event_manager/EventManager.hpp"

namespace atrias {

namespace eventManager {

EventManager::EventManager(std::string const &name) :
	RTT::TaskContext(name),
	cmOut("cm_events_out"),
	eventsIn("events_in"),
	guiOut("gui_events_out")
{
	this->addPort(cmOut);

	// Lets make this an event port, so \a updateHook() will
	// get called when we receive a new event.
	this->addEventPort(eventsIn);
	
	this->addPort(guiOut);

	return;
}

void EventManager::sendCM(atrias_msgs::rt_ops_event &event) {
	cmOut.write(event);
}

void EventManager::sendGUI(atrias_msgs::rt_ops_event &event) {
	guiOut.write(event);
}

void EventManager::updateHook() {
	// Check if a new event has arrived.
	atrias_msgs::rt_ops_event newEvent;
	if (eventsIn.read(newEvent)) {
		// Hey, a new event *has* arrived!
		processEvent(newEvent);
	}
}

void EventManager::processEvent(atrias_msgs::rt_ops_event &event) {
	switch ((rtOps::RtOpsEvent) event.event) {
		case rtOps::RtOpsEvent::MISSED_DEADLINE:
			log(RTT::Warning) << "[EventManager] Missed deadline event received." << RTT::endlog();
			sendGUI(event);
			break;

		case rtOps::RtOpsEvent::ACK_CM:
			sendCM(event);
			break;

		case rtOps::RtOpsEvent::ACK_GUI: // These all intentionally fall through; their behavior is all the same.
		case rtOps::RtOpsEvent::SAFETY:
		case rtOps::RtOpsEvent::CONTROLLER_CUSTOM:
		case rtOps::RtOpsEvent::RTOPS_STATE_CHG:
		case rtOps::RtOpsEvent::GUI_STATE_CHG:
		case rtOps::RtOpsEvent::CONT_STATE_CHG:
			sendGUI(event);
			break;

		default:
			// If we don't know what it is, then we should probably send it to both,
			// in case one of them does recognize it.
			sendCM(event);
			sendGUI(event);
			break;
	}
}

}

}

ORO_CREATE_COMPONENT(atrias::eventManager::EventManager);

// vim: noexpandtab
