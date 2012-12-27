#ifndef EVENTMANAGER_HPP
#define EVENTMANAGER_HPP

/** @file EventManager.hpp
  * @brief This contains the main class for the Event Manager component.
  */

// Orocos
#include <rtt/Component.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>

// Atrias
#include <atrias_msgs/rt_ops_event.h>
#include <atrias_shared/globals.h>

namespace atrias {

namespace eventManager {

class EventManager : public RTT::TaskContext {
	public:
		/** @brief Initializes this component.
		  * @param name This component's name.
		  */
		EventManager (std::string const &name);

		/** @brief This forwards an event to the Controller Manager.
		  * @param event The event to be forwarded.
		  */
		void sendCM(atrias_msgs::rt_ops_event &event);

		/** @brief This forwards an event to the GUI.
		  * @param event The event to be forwarded.
		  */
		void sendGUI(atrias_msgs::rt_ops_event &event);

		/** @brief This is called by Orocos when something (like a new event) needs our attention.
		  */
		void updateHook();
	
	private:
		/** @brief This is the event output port to the Controller Manager.
		  */
		RTT::OutputPort<atrias_msgs::rt_ops_event> cmOut;

		/** @brief This is how we receive events from RT Ops.
		  */
		RTT::InputPort<atrias_msgs::rt_ops_event> eventsIn;

		/** @brief This is the output port to the GUI.
		  */
		RTT::OutputPort<atrias_msgs::rt_ops_event> guiOut;

		/** @brief This processes a new event.
		  * @param event The event to be processed.
		  */
		void processEvent(atrias_msgs::rt_ops_event &event);
};

}

}

#endif // EVENTMANAGER_HPP

// vim: noexpandtab
