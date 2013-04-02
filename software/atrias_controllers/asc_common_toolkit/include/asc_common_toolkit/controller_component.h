#ifndef __ASC_COMMON_TOOLKIT_H__
#define __ASC_COMMON_TOOLKIT_H__

/*! \file controller_component.h
 *  \author Mikhail Jones
 *  \brief Orocos Component header for the asc_common_toolkit subcontroller.
 */

// Orocos 
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

// C
#include <tuple>
#include <stdlib.h>
#include <robot_invariant_defs.h>

// Datatypes
#include <asc_common_toolkit/controller_log_data.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_common_toolkit;

namespace atrias {
namespace controller {


// ASCCommonToolkit ============================================================
class ASCCommonToolkit : public TaskContext {

	private:
		// Operations ----------------------------------------------------------
		double legStiffness(double r, double r0);
		std::tuple<double, double> polMotorPos2LegPos(double qmA, double qmB);
		std::tuple<double, double> polLegPos2MotorPos(double ql, double rl);
		std::tuple<double, double> polLegVel2MotorVel(double ql, double dql, double drl);
		

		// Variables -----------------------------------------------------------
		// Virtual leg length stiffness
		double k;
		
		// Motor positions
		double qmA, qmB;
		
		// Motor velocities
		double dqmA, dqmB;
		
		// Leg positions
		double ql, rl;
		

		// Logging -------------------------------------------------------------
		controller_log_data logData;
		OutputPort<controller_log_data> logPort;
		

	public:
		// Constructor ---------------------------------------------------------
		ASCCommonToolkit(std::string name);

    	// Get ROS header from RTOps -------------------------------------------
    	RTT::OperationCaller<std_msgs::Header(void)> getROSHeader;
    	
		// Standard Orocos hooks -----------------------------------------------
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		
}; // class ASCCommonToolkit

} // namespace controller
} // namespace atrias

#endif
