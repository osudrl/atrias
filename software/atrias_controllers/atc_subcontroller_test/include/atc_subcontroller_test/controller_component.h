#ifndef __ATC_SUBCONTROLLER_TEST_H__
#define __ATC_SUBCONTROLLER_TEST_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for atc_subcontroller_test controller.
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

// Our stuff
#include <atc_subcontroller_test/controller_input.h>
#include <atc_subcontroller_test/controller_status.h>
#include <atc_subcontroller_test/controller_log_data.h>
#include <atrias_asc_loader/ASCLoader.hpp>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_subcontroller_test;

namespace atrias {
using namespace shared;
namespace controller {

class ATCSubcontrollerTest : public TaskContext {
	private:
		// This Operation is called by the RT Operations Manager.
		atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);
		
		// Logging
		controller_log_data logData;
		OutputPort<controller_log_data>  logPort;

		// The subcontroller -- will need modification.
		ASCLoader  controllerLoader;
		Property<double> subcontrollerProperty;
		OperationCaller<double(double)> subcontrollerOperationCaller;
		
		// For the GUI
		shared::GuiPublishTimer                         *pubTimer;
		controller_input guiIn;
		controller_status guiOut;
		OutputPort<controller_status>                   guiDataOut;
		InputPort<controller_input>                     guiDataIn;
		
	public:
		// Constructor
		ATCSubcontrollerTest(std::string name);
		
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
