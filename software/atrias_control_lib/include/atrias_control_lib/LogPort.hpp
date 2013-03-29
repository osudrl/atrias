#ifndef LOGPORT_HPP
#define LOGPORT_HPP

/**
  * @file LogPort.hpp
  * @author Ryan Van Why
  * @brief This class allows subcontrollers to log data.
  * This may also be utilized by top-level controllers
  * for additional log ports.
  */

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
		LogPort(const AtriasController *controller, const std::string name) {
			// TODO: Implement
		}

		/**
		  * @brief This allows controllers to access the data to be logged.
		  */
		logType data;

		/**
		  * @brief This transmits the data for logging.
		  * This will not alter the data itself.
		  */
		void send() {
			// TODO: Implement
		}
};

}
}

#endif // LOGPORT_HPP

// vim: noexpandtab
