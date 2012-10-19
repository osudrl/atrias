#ifndef __ATC_MOTOR_TEST_H__
#define __ATC_MOTOR_TEST_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_motor_test controller.
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
#include <atc_motor_test/controller_input.h>
#include <atc_motor_test/controller_status.h>
#include <atc_motor_test/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_motor_test;

namespace atrias {
using namespace shared;
namespace controller {

class ATCMotorTest : public TaskContext {
	private:
		// This Operation is called by the RT Operations Manager.
		atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);
		
	public:
		// Constructor
		ATCMotorTest(std::string name);

		// Standard Orocos hooks
		bool configureHook();
		bool startHook();
		void stopHook();
		void cleanupHook();
};
}
}

#endif
