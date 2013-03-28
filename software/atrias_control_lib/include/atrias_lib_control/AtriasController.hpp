#ifndef ATRIASCONTROLLER_HPP
#define ATRIASCONTROLLER_HPP

/**
  * @file AtriasController.hpp
  * @author Ryan Van Why
  * @brief This class is subclassed by all atrias controllers.
  * This is subclassed directly by subcontrollers, and indirectly
  * through ATC for top-level controllers.
  */

// For names, and possibly a realtime debugging system in the future
#include <string>

// Orocos
#include <rtt/TaskContext.hpp> // We're not a TaskContext, but we need to reference one.

// Our namespaces
namespace atrias {
namespace controller {

// Subcontrollers do not need to be components, so this is not a TaskContext.
class AtriasController {
	public:
		/**
		  * @brief The constructor for this class.
		  * @param parent The parent controller of this subcontroller.
		  * @param name   This controller's name.
		  * If this is a TLC, the parent should be this instance and
		  * the name the component's name.
		  */
		AtriasController(const AtriasController &parent, const std::string &name);

		/**
		  * @brief This returns this controller's name.
		  * @return The name of this controller.
		  * This name includes the full hierarchy, not just the name passed in.
		  */
		const std::string& getName() const;

	private:
		/**
		  * @brief This returns the TaskContext
		  * @return A reference to the TaskContext.
		  */
		RTT::TaskContext& getTaskContext() const;

		// This controller's (full) name
		std::string name;

		// A reference to the top-level controller
		RTT::TaskContext &tc;
};

}
}

#endif // ATRIASCONTROLLER_HPP

// vim: noexpandtab
