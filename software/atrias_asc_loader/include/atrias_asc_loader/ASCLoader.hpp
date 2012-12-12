#ifndef ASCLOADER_HPP
#define ASCLOADER_HPP

/** @file   ASCLoader.hpp
  * @author Ryan Van Why
  * @brief  This class makes loading and unloading subcontrollers
  *         as easy as possible.
  */

// Standard libs
#include <string>

// Orocos
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>

namespace atrias {
namespace controller {

class ASCLoader {
	public:
		/** @brief Sets up this ASCLoader instance
		  */
		ASCLoader();

		/** @brief              Loads and configures this subcontroller
		  * @param task_context A pointer to this component's TaskContext, for interacting
		  *                     with the deployer and other components.
		  * @param package      The package in which this subcontroller resides.
		  * @param type         The name of the main class of this subcontroller
		  * @return             A pointer to the newly-loaded component, or nullptr on error.
		  */
		RTT::TaskContext* load(RTT::TaskContext* task_context, std::string package, std::string type);

		/** @brief Cleans up this class and the subcontroller
		  */
		~ASCLoader();
	
	private:
		/** @brief The deployer. Lets us do lots of stuff.
		  */
		RTT::TaskContext* deployer;

		/** @brief Outputs an error message to the console.
		  * @param message      The specific error message for the console
		  * @param task_context The TaskContext pointer passed in
		  * @param package      The package passed in
		  * @param type         The type passed in
		  * Used during component loading
		  */
		void reportLoadError(std::string       message,
		                     RTT::TaskContext* task_context,
		                     std::string       package,
		                     std::string       type);

		/** @brief Outputs an error message to the console.
		  * @param message The error message
		  * Used during component unloading.
		  * subcontroller must not be NULL here!
		  */
		void reportUnloadError(std::string message);

		/** @brief The component's name. Needed to unload the component.
		  */
		std::string name;

		/** @brief This references our subcontroller's component.
		  * Used to access it for init and shutdown.
		  */
		RTT::TaskContext* subcontroller;
};

}
}

#endif // ASCLOADER_HPP

// vim: noexpandtab
