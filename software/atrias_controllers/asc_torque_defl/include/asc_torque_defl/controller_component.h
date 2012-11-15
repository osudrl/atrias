#ifndef __ASC_TORQUE_DEFL_H__
#define __ASC_TORQUE_DEFL_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for the asc_torque_defl subcontroller.
 */

// Orocos
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>

#include <robot_invariant_defs.h>

// Datatypes
#include <asc_torque_defl/controller_log_data.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_torque_defl;

namespace atrias {
namespace controller {

class ASCTorqueDefl : public TaskContext {
	private:
		// Operations
		double getDefl(double torque);
		
		// Subcontroller names
		std::string linearInterp0Name;
		
		// Subcontroller components
		TaskContext *linearInterp0;
		
		// Service properties
		
		// Subcontroller operations
		OperationCaller<void(double samples[], int, double, double)> linearInterp0InputPoints;
		OperationCaller<double(double)> linearInterp0GetValue;
		
		// Logging
		OutputPort<controller_log_data> logPort;
		
	public:
		// Constructor
		ASCTorqueDefl(std::string name);
		
		// Standard Orocos hooks
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}
}

#endif
