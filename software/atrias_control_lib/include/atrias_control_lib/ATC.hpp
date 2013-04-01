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
#include <rtt/InputPort.hpp>   // So we can receive data.
#include <rtt/TaskContext.hpp> // We're a component aka TaskContext

// Robot state and controller output
#include <atrias_msgs/controller_output.h>
#include <atrias_msgs/robot_state.h>

// This lets us check whether to create/check ports
#include <atrias_shared/notNullPtr.hpp>

// For the medulla state enum
#include <../../robot_definitions/robot_invariant_defs.h>

// We subclass this, so let's include it
#include "atrias_control_lib/AtriasController.hpp"

// Our namespaces
namespace atrias {
namespace controller {

// This is a component, so we subclass TaskContext;
// as a controller, this subclasses AtriasController
// Also, we're a template...
template <typename logType = std::nullptr_t, typename guiInType = std::nullptr_t, typename guiOutType = std::nullptr_t>
class ATC : public RTT::TaskContext, public AtriasController {
	public:
		/**
		  * @brief The constructor for this class.
		  * @param name This component's name.
		  * The name should be passed directly -- it's not the controller's
		  * choice.
		  */
		ATC(const std::string &name);

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
		RTT::TaskContext& getTaskContext() const;

		// Port for input data from the GUI
		RTT::InputPort<guiInType> guiInPort;

		/**
		  * @brief This callback is executed when data is received from the GUI
		  */
		void guiInCallback(RTT::base::PortInterface* portInterface);

		/**
		  * @brief This is the operation called cyclically by RT Ops
		  * This runs the controller.
		  * @param robotState The new robot state
		  * @return           The new controller output.
		  */
		atrias_msgs::controller_output& runController(atrias_msgs::robot_state& robotState);
};

template <typename logType, typename guiInType, typename guiOutType>
ATC<logType, guiInType, guiOutType>::ATC(const std::string &name) :
	RTT::TaskContext(name),
	AtriasController(name)
{
	// Register the operation runController()
	this->provides("atc")
		->addOperation("runController", &ATC<logType, guiInType, guiOutType>::runController, this, RTT::ClientThread)
		.doc("Run the controller. Takes in the robot state and returns a controller output.");

	// Set up the event port for incoming GUI data (if there is incoming GUI data)
	if (atrias::shared::notNullPtr<guiInType>())
		this->addEventPort("Gui Input", guiInPort, boost::bind(&ATC<logType, guiInType, guiOutType>::guiInCallback, this, _1));
}

template <typename logType, typename guiInType, typename guiOutType>
RTT::TaskContext& ATC<logType, guiInType, guiOutType>::getTaskContext() const {
	return *((RTT::TaskContext*) this);
}

template <typename logType, typename guiInType, typename guiOutType>
void ATC<logType, guiInType, guiOutType>::guiInCallback(RTT::base::PortInterface* portInterface) {
	this->guiInPort.read(this->guiIn);
}

template <typename logType, typename guiInType, typename guiOutType>
atrias_msgs::controller_output& ATC<logType, guiInType, guiOutType>::runController(atrias_msgs::robot_state& robotState) {
	this->rs = robotState;
	this->controller();
	this->co.command = medulla_state_run;
	return this->co;
}

}
}

#endif // ATC_HPP

// vim: noexpandtab
