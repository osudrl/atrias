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

// ROS
#include <std_msgs/Header.h> // So we can pass around ROS headers for logging.

// Our namespaces
namespace atrias {
namespace controller {

// Subcontrollers do not need to be components, so this is not a TaskContext.
class AtriasController {
	protected:
		/**
		  * @brief The normal constructor for this class.
		  * @param parent The parent controller of this subcontroller.
		  * @param name   This controller's name.
		  * This is the constructor that subcontrollers should use.
		  */
		AtriasController(const AtriasController * const parent, const std::string &name);

		/**
		  * @brief The ATC constructor for this class.
		  * @param name This component's name.
		  * This should only be called by the ATC constructor.
		  */
		AtriasController(const std::string &name);

		/**
		  * @brief This returns the value num clamped between min and max.
		  * @param num The number to be clamped
		  * @param min The minimum output value.
		  * @param max The maximum output value.
		  * @return The clamped value (>=min, <=max)
		  * This is a convenience function for controllers. This will work on any
		  * type with a defined '<' comparison operator
		  */
		template <typename T>
		const T& clamp(const T& num, const T& min, const T& max) const {
			if (num < min)
				return min;

			if (max < num)
				return max;

			return num;
		}

	public:
		/**
		  * @brief This returns this controller's name.
		  * @return The name of this controller.
		  * This name includes the full hierarchy, not just the name passed in.
		  */
		const std::string& getName() const;

		/**
		  * @brief Returns a ROS header with the current timestamp.
		  * @return A ROS header for logging purposes.
		  */
		virtual const std_msgs::Header& getROSHeader() const;

		/**
		  * @brief This returns the TaskContext
		  * @return A reference to the TaskContext.
		  * This should only be overridden by the ATC class
		  */
		virtual RTT::TaskContext& getTaskContext() const;

	private:
		/**
		  * @brief This returns the TLC as an AtriasController
		  * @return A reference to the top-level controller.
		  */
		AtriasController &getTLC() const;

		// This controller's (full) name
		std::string name;

		// A reference to the top-level controller as an AtriasController
		AtriasController &tlc;
};

}
}

#endif // ATRIASCONTROLLER_HPP

// vim: noexpandtab
