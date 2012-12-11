#ifndef COMPONENENTLOADER_HPP
#define COMPONENENTLOADER_HPP

/** @file   ComponentLoader.hpp
  * @author Ryan Van Why
  * @brief  This class makes loading and unloading our components
  *         as easy as possible.
  */

// Standard libs
#include <string>

// Orocos
#include <rtt/TaskContext.hpp>

namespace atrias {
namespace ComponentLoader {

class ComponentLoader {
	public:
		/** @brief Allows access to the created component
		  * @return A pointer to this component
		  */
		RTT::TaskContext* getTaskContext();

		/** @brief              Loads and configures this component
		  * @param task_context A pointer to this component's TaskContext, for interacting
		  *                     with the deployer and other components.
		  * @param type         The name of the main class of this component
		  */
		ComponentLoader(RTT::TaskContext* task_context, std::string type);

		/** @brief Cleans up this class and the component
		  */
		~ComponentLoader();
};

}
}

#endif // COMPONENENTLOADER_HPP

// vim: noexpandtab
