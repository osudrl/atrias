#ifndef SUBCONTROLLER_HPP
#define SUBCONTROLLER_HPP

/** @file SubController.hpp
  * @author Ryan Van Why
  * @brief This class represents a subcontroller to
  *        higher-level controllers.
  */

// Standard libs
#include <string>

// Orocos
#include <rtt/TaskContext.hpp>

namespace atrias {
namespace controller {

class SubController {
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
		SubController(RTT::TaskContext* task_context, std::string type);

		/** @brief Cleans up this class and the subcontroller.
		  */
		~SubController();
};

}
}

#endif // SUBCONTROLLER_HPP

// vim: noexpandtab
