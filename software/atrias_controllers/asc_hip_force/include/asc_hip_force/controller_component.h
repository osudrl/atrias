#ifndef __ASC_HIP_FORCE_H__
#define __ASC_HIP_FORCE_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for the asc_hip_force subcontroller.
 */

// Orocos
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>

// Our stuff
#include <asc_hip_force/controller_log_data.h>
#include <atrias_asc_loader/ASCLoader.hpp>
#include <robot_invariant_defs.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_hip_force;

namespace atrias {
namespace controller {

class ASCHipForce : public TaskContext {
	private:
		// Operations
		double runController(double exampleInput);

		double out;

		// Logging
		controller_log_data logData;
		OutputPort<controller_log_data> logPort;

	public:
		// Constructor
		ASCHipForce(std::string name);

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

// vim: noexpandtab
