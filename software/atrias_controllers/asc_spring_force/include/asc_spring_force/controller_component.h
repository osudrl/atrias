#ifndef __ASC_SPRING_FORCE_H__
#define __ASC_SPRING_FORCE_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for the asc_spring_force subcontroller.
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
#include <asc_spring_force/controller_log_data.h>
#include <atrias_asc_loader/ASCLoader.hpp>

using namespace RTT;
using namespace Orocos;
using namespace asc_spring_force;

namespace atrias {
namespace controller {

class ASCSpringForce : public TaskContext {
	private:
		// Operations
		double getForce(double motorAAngle, double legAAngle, double motorBAngle, double legBAngle);
		
		ASCLoader springTorque0Loader;
		ASCLoader springTorque1Loader;
		
		// Subcontroller operations
		OperationCaller<double(double)> springTorque0GetTorque;
		OperationCaller<double(double)> springTorque1GetTorque;
		
		// Logging
		OutputPort<controller_log_data> logPort;
		
	public:
		// Constructor
		ASCSpringForce(std::string name);
		
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
