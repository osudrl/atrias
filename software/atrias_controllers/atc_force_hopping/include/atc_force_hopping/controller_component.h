#ifndef __ATC_FORCE_HOPPING_H__
#define __ATC_FORCE_HOPPING_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for atc_force_hopping controller.
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

#include <atrias_shared/GuiPublishTimer.h>
#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>

// Datatypes
#include "atc_force_hopping/controller_common.hpp"
#include <atc_force_hopping/controller_input.h>
#include <atc_force_hopping/controller_status.h>
#include <atc_force_hopping/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/globals.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_force_hopping;

namespace atrias {
using namespace shared;
namespace controller {

class ATCForceHopping : public TaskContext {
	private:
		// This Operation is called by the RT Operations Manager.
		atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

		State mode;

		// This lets us send an event upon encountering an error
		OperationCaller<void(rtOps::RtOpsEvent, rtOps::RtOpsEventMetadata_t)> sendEvent;
		
		// Logging
		OutputPort<controller_log_data>  logPort;
		
		// For the GUI
		shared::GuiPublishTimer                         *pubTimer;
		controller_input guiIn;
		InputPort<controller_input>                     guiDataIn;
		
	public:
		// Constructor
		ATCForceHopping(std::string name);
		
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
