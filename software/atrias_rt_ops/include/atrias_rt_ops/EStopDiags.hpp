// Standard include guard
#ifndef ESTOPDIAGS_HPP
#define ESTOPDIAGS_HPP

/**
  * @file EStopDiags.hpp
  * @author Ryan Van Why
  * @brief This class prints out some diagnostic information
  *        in a realtime-safe manner when an EStop occurs.
  */

// This forward declaration is necessary to make RTOps compile
namespace atrias {
namespace rtOps {
class EStopDiags;
}
}

// ATRIAS project includes
#include "RTOps.h" // We need to modify the RTOps component to add our operations.

// Namespaces for the ATRIAS project and the RT Ops component
namespace atrias {
namespace rtOps {

class EStopDiags: public RTT::Service {
	public:
		/**
		  * @brief The constructor for this class.
		  * @param rt_ops A pointer to the main RT Ops instance
		  * This initializes the class, including setting up the operation and
		  * operation caller
		  */
		EStopDiags(RTOps *rt_ops);

		/**
		  * @brief This prints out information about an estop (RT-safe)
		  * @param event The event for this estop
		  * This calls the backend function which does the actual work of printing
		  * (to prevent executing non-deterministic functions)
		  */
		void printEStop(RtOpsEvent &event);

	private:
		/**
		  * @brief This is the operation that prints out EStop events
		  * @param event The EStop event to be printed out
		  */
		void printEStopBackend(RtOpsEvent event);

		// OperationCaller for the above operation
		RTT::OperationCaller<void(RtOpsEvent event)> printEStopCaller;
};

// End namespaces
}
}

#endif // ESTOPDIAGS_HPP

// Configure vim for tab-based indentation
// vim: noexpandtab
