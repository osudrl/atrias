#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

/** @file   Controller.hpp
  * @author Ryan Van Why
  * @brief  This class represents a controller to
  *         higher-level controllers.
  */

// Standard libs
#include <string>

// Orocos
#include <rtt/TaskContext.hpp>

namespace atrias {
namespace controller {

class Controller {
	public:
		/** @brief Allows the controller to access this subcontroller
		  * @return A pointer to this subcontroller
		  */
		RTT::TaskContext* getTaskContext();

		/** @brief              Loads and configures this subcontroller.
		  * @param task_context A pointer to this component's TaskContext, for interacting
		  *                     with the deployer and other components.
		  * @param type         The name of the main class of this subcontroller.
		  */
		Controller(RTT::TaskContext* task_context, std::string type);

		/** @brief Cleans up this class and the subcontroller.
		  */
		~Controller();
};

}
}

#endif // CONTROLLER_HPP

// vim: noexpandtab
