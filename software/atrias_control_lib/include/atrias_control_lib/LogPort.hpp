#ifndef LOGPORT_HPP
#define LOGPORT_HPP

/**
  * @file LogPort.hpp
  * @author Ryan Van Why
  * @brief This class allows subcontrollers to log data.
  * This may also be utilized by top-level controllers
  * for additional log ports.
  */

// Orocos includes
#include <rtt/ConnPolicy.hpp> // Allows for connecting the output port to ROS
#include <rtt/OutputPort.hpp> // This allows the creation of an output port

// This allows us to access the name and TaskContext
#include "atrias_control_lib/AtriasController.hpp"

namespace atrias {
namespace controller {

template <typename logType>
class LogPort {
	public:
		/**
		  * @brief The constructor for this logging port.
		  * @param controller A reference to this controller
		  * @param name       The name for this log port.
		  */
		LogPort(const AtriasController* const controller, const std::string name = "log");

		/**
		  * @brief This allows controllers to access the data to be logged.
		  */
		logType data;

		/**
		  * @brief This transmits the data for logging.
		  * This will not alter the data itself.
		  */
		void send();
	
	private:
		// Our output port
		RTT::OutputPort<logType> port;

		// Allows us to access the top-level controller.
		const AtriasController &tlc;
};

template <typename logType>
LogPort<logType>::LogPort(const AtriasController* const controller, const std::string name) :
	port(controller->getName() + "_" + name),
	tlc(controller->getTLC())
{
	// Setup our port
	this->tlc.getTaskContext().addPort(this->port);

	RTT::ConnPolicy policy = RTT::ConnPolicy::buffer(10000);
	// Transport 3 is ROS
	policy.transport = 3;
	// Set the topic name
	policy.name_id = "/" + controller->getName() + "_" + name;
	// And actually initiate the connection
	this->port.createStream(policy);
}

template <typename logType>
void LogPort<logType>::send() {
	// Set the timestamp
	this->data.header = this->tlc.getROSHeader();

	// Send the data
	this->port.write(this->data);
}

}
}

#endif // LOGPORT_HPP

// vim: noexpandtab
