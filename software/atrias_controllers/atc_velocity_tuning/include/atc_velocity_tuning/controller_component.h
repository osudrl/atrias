#ifndef __ATC_VELOCITY_TUNING_H__
#define __ATC_VELOCITY_TUNING_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for atc_velocity_tuning controller.
 */

// Orocos
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>

#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>

// Datatypes
#include <atc_velocity_tuning/controller_input.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_velocity_tuning;

namespace atrias {
namespace controller {

class ATCVelocityTuning : public TaskContext {
	private:
		// This Operation is called by the RT Operations Manager.
		atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);
		
		// Subcontroller names
		std::string pd0Name;
		
		// Subcontroller components
		TaskContext *pd0;
		
		// Service properties
		Property<double> D0;
		Property<double> P0;
		
		// Subcontroller operations
		OperationCaller<double(double, double, double, double)> pd0RunController;
		
		int cur_dir;
		
		// For the GUI
		controller_input guiIn;
		InputPort<controller_input>                     guiDataIn;
		
	public:
		// Constructor
		ATCVelocityTuning(std::string name);
		
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
