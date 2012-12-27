#ifndef EVENTMANAGER_HPP
#define EVENTMANAGER_HPP

/** @file EventManager.hpp
  * @brief This contains the main class for the Event Manager component.
  */

// Orocos
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>

namespace atrias {

namespace eventManager {

class EventManager : public RTT::TaskContext {
	public:
		/** @brief Initializes this component.
		  * @param name This component's name.
		  */
		EventManager (std::string const &name);
};

}

}

#endif // EVENTMANAGER_HPP

// vim: noexpandtab
