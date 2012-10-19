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
#include <rtt/os/TimeService.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>
#include <atrias_shared/drl_math.h>
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

#define MAX_TORQUE       0.7
#define MIN_TORQUE       0.4
#define MIN_DURATION_SEC 1.0
#define MAX_DURATION_SEC 10.0

namespace atrias {
namespace controller {

class ATCMotorTest : public TaskContext {
	private:
		// This Operation is called by the RT Operations Manager.
		atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);
		
		RTT::os::TimeService::nsecs nextSwitchTime;
		
		atrias_msgs::controller_output co;
		
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
