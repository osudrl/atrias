#include "atrias_event_manager/EventManager.hpp"

namespace atrias {

namespace eventManager {

EventManager::EventManager(std::string const &name) :
	RTT::TaskContext(name)
{
	return;
}

}

}

ORO_CREATE_COMPONENT(atrias::eventManager::EventManager);

// vim: noexpandtab
