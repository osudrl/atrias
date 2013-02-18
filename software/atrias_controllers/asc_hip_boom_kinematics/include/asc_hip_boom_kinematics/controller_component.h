#ifndef __ASC_HIP_BOOM_KINEMATICS_H__
#define __ASC_HIP_BOOM_KINEMATICS_H__

/*! \file controller_component.h
 *  \author Mikhail Jones
 *  \brief Orocos Component header for the asc_hip_boom_kinematics subcontroller.
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
#include <complex.h>

// Datatypes
#include <asc_hip_boom_kinematics/controller_log_data.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_msgs/robot_state.h>


using namespace RTT;
using namespace Orocos;
using namespace asc_hip_boom_kinematics;
using namespace std; // might not need

namespace atrias {
namespace controller {


// ASCHipBoomKinematics ========================================================
class ASCHipBoomKinematics : public TaskContext {
	private:
    	// Operations
    	LeftRight inverseKinematics(LeftRight toePosition, atrias_msgs::robot_state_leg lLeg, atrias_msgs::robot_state_leg rLeg, atrias_msgs::robot_state_location position);

    	// Function Variables
       	complex<double> i;
    	complex<double> complexHipAngleLeft;
    	complex<double> complexHipAngleRight;
		double l1, l2;
		double lBoom, lBody, lHip, qBodyOffset;
		double lLeftLeg, lRightLeg, qLeftLeg, qRightLeg;
        LeftRight hipAngle;

    	// Logging
    	controller_log_data logData;
    	OutputPort<controller_log_data> logPort;

	public:
    	// Constructor
    	ASCHipBoomKinematics(std::string name);
    	
    	// Get ROS header from RTOps.
    	RTT::OperationCaller<std_msgs::Header(void)> getROSHeader;

	    // Standard Orocos hooks
    	bool configureHook();
    	bool startHook();
    	void updateHook();
    	void stopHook();
    	void cleanupHook();
    	
}; // class ASCHipBoomKinematics

} // namespace controller
} // namespace atrias

#endif
