#ifndef __ASC_LINEAR_INTERP_H__
#define __ASC_LINEAR_INTERP_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for the asc_linear_interp subcontroller.
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
#include <asc_linear_interp/controller_log_data.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_linear_interp;

namespace atrias {
namespace controller {

class ASCLinearInterp : public TaskContext {
	private:
		// Operations
		double runController(double input);
		
		// Logging
		OutputPort<controller_log_data> logPort;
		
	public:
		// Constructor
		ASCLinearInterp(std::string name);
		
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
