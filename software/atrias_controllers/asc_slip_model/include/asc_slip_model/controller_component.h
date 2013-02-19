#ifndef __ASC_SLIP_MODEL_H__
#define __ASC_SLIP_MODEL_H__

/*! \file controller_component.h
 *  \author Mikhail Jones
 *  \brief Orocos Component header for the asc_slip_model subcontroller.
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
#include <asc_slip_model/controller_log_data.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_slip_model;
using namespace std; // might not need

namespace atrias {
namespace controller {

// ASCSlipModel ================================================================
class ASCSlipModel : public TaskContext {
	private:
	    // Operations
    	SlipConditions slipAdvance(SlipModel slipModel, SlipConditions slipConditions);
		LegForce slipForce(SlipModel slipModel, SlipConditions slipConditions);
		
		// Function variables
		double delta;
		LegForce legForce;

    	// Logging
    	controller_log_data logData;
    	OutputPort<controller_log_data> logPort;

	public:
    	// Constructor
    	ASCSlipModel(std::string name);
    	
    	// Get ROS header from RTOps.
    	RTT::OperationCaller<std_msgs::Header(void)> getROSHeader;

    	// Standard Orocos hooks
    	bool configureHook();
    	bool startHook();
    	void updateHook();
    	void stopHook();
    	void cleanupHook();
    	
}; // class ASCSlipModel

} // namespace controller
} // namespace atrias

#endif
