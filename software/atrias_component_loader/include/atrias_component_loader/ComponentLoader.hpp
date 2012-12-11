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
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>

namespace atrias {
namespace ComponentLoader {

class ComponentLoader {
	public:
		/** @brief Sets up this ComponentLoader
		  */
		ComponentLoader();

		/** @brief              Loads and configures this component
		  * @param task_context A pointer to this component's TaskContext, for interacting
		  *                     with the deployer and other components.
		  * @param package      The package in which this component resides.
		  * @param type         The name of the main class of this component
		  * @return             A pointer to the newly-loaded component, or nullptr on error.
		  */
		RTT::TaskContext* loadComponent(RTT::TaskContext* task_context, std::string package, std::string type);

		/** @brief Cleans up this class and the component
		  */
		~ComponentLoader();
	
	private:
		/** @brief This references our component. Used to access it for init and shutdown.
		  */
		RTT::TaskContext* component;

		/** @brief The deployer. Lets us do lots of stuff.
		  */
		RTT::TaskContext* deployer;

		/** @brief The component's name. Needed to unload the component.
		  */
		std::string name;
};

}
}

#endif // COMPONENENTLOADER_HPP

// vim: noexpandtab
