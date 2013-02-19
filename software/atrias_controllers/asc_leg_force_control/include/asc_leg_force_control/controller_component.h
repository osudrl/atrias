#ifndef __ASC_LEG_FORCE_CONTROL_H__
#define __ASC_LEG_FORCE_CONTROL_H__

/*! \file controller_component.h
 *  \author Mikhail Jones
 *  \brief Orocos Component header for the asc_leg_force_control subcontroller.
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
#include <asc_leg_force_control/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_leg_force_control;
using namespace std; // might not need

namespace atrias {
namespace controller {


// ASCLegForceControl ==========================================================
class ASCLegForceControl : public TaskContext {
	private:
        // Operations
        AB legForceToMotorCurrent(LegForce legForce, Gain gain, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position);

		// Function variables
		double l1, l2;
		double tauSpringA, tauSpringB;
		double dtauSpringA, dtauSpringB;
		AB motorCurrent;
		
    	// Logging
    	controller_log_data logData;
    	OutputPort<controller_log_data> logPort;

	public:
    	// Constructor
    	ASCLegForceControl(std::string name);
    	
    	// Get ROS header from RTOps.
    	RTT::OperationCaller<std_msgs::Header(void)> getROSHeader;

    	// Standard Orocos hooks
    	bool configureHook();
    	bool startHook();
    	void updateHook();
    	void stopHook();
    	void cleanupHook();
    	
}; // class ASCLegForceControl

} // namespace controller
} // namespace atrias

#endif
