#ifndef __ASC_FORCE_CONTROL_H__
#define __ASC_FORCE_CONTROL_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for the asc_force_control subcontroller.
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

// Our Stuff
#include <asc_force_control/controller_log_data.h>
#include <atrias_asc_loader/ASCLoader.hpp>
#include <atrias_msgs/robot_state_leg.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_force_control;
using namespace atrias_msgs;

namespace atrias {
namespace controller {

class ASCForceControl : public TaskContext {
	private:
		// Operations
		LegState getTgtState(atrias_msgs::robot_state_leg legState, double tgtForce, double dTgtForce);
		
		// Subcontrollers
		ASCLoader torqueDeflALoader;
		ASCLoader torqueDeflBLoader;
		
		// Subcontroller operations
		OperationCaller<double(double)> sprTrqAConstant;
		OperationCaller<double(double)> sprTrqADefl;
		OperationCaller<double(double)> sprTrqBConstant;
		OperationCaller<double(double)> sprTrqBDefl;
		
		// Logging
		OutputPort<controller_log_data> logPort;
		
	public:
		// Constructor
		ASCForceControl(std::string name);
		
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
