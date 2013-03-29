#ifndef ATC_HPP
#define ATC_HPP

/**
  * @file ATC.hpp
  * @author Ryan Van Why
  * @brief This should be subclassed by top-level controllers.
  * This provides interfaces to make writing controllers easier.
  */

// Standard library
#include <cstddef> // nullptr_t

// Orocos includes
#include <rtt/TaskContext.hpp> // We're a component aka TaskContext

// Robot state and controller output
#include <atrias_msgs/controller_output.h>
#include <atrias_msgs/robot_state.h>

// We subclass this, so let's include it
#include "atrias_control_lib/AtriasController.hpp"

// Our namespaces
namespace atrias {
namespace controller {

// This is a component, so we subclass TaskContext;
// as a controller, this subclasses AtriasController
// Also, we're a template...
template <typename logType, typename guiInType, typename guiOutType>
class ATC : public RTT::TaskContext, public AtriasController {
	public:
		/**
		  * @brief The constructor for this class.
		  * @param name This component's name.
		  * The name should be passed directly -- it's not the controller's
		  * choice.
		  */
		ATC(const std::string &name) :
			RTT::TaskContext(name),
			AtriasController(name)
		{
			// The magic is done above
		}

	protected:
		// These member variables should be set/read from by
		// the controllers themselves.
		logType    logOut;
		guiInType  guiIn;
		guiOutType guiOut;

		// Here is the robot state
		atrias_msgs::robot_state rs;

		// And the controller output
		atrias_msgs::controller_output co;

	private:
		/**
		  * @brief This is the actual controller function.
		  * This should be overloaded by the actual controller.
		  */
		virtual void controller() = 0;

		/**
		  * @brief This allows subcontrollers to access the TaskContext
		  * @return A reference to the TaskContext.
		  */
		RTT::TaskContext& getTaskContext() const {
			return *((RTT::TaskContext*) this);
		}
};

}
}

#endif // ATC_HPP

// vim: noexpandtab
