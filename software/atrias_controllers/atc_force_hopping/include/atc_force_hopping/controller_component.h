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
#include "atc_force_hopping/common.hpp"
#include <atc_force_hopping/controller_input.h>
#include <atc_force_hopping/controller_status.h>
#include <atc_force_hopping/controller_log_data.h>
#include <atrias_asc_loader/ASCLoader.hpp>
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
		State mode;

		// This lets us send an event upon encountering an error
		OperationCaller<void(rtOps::RtOpsEvent, rtOps::RtOpsEventMetadata_t)> sendEvent;
		
		// Logging
		OutputPort<controller_log_data>  logPort;
		
		// For the GUI
		shared::GuiPublishTimer         *pubTimer;
		controller_input                guiIn;
		controller_status               guiOut;
		InputPort<controller_input>     guiDataIn;
		OutputPort<controller_status>   guiDataOut;

		// Our subcontrollers
		ASCLoader lLegAControllerLoader;
		ASCLoader lLegBControllerLoader;
		ASCLoader lLegHControllerLoader;
		ASCLoader rLegAControllerLoader;
		ASCLoader rLegBControllerLoader;
		ASCLoader rLegHControllerLoader;
		ASCLoader lLegASmoothLoader;
		ASCLoader lLegBSmoothLoader;
		ASCLoader lLegHSmoothLoader;
		ASCLoader rLegASmoothLoader;
		ASCLoader rLegBSmoothLoader;
		ASCLoader rLegHSmoothLoader;

		OperationCaller<double(double, double, double, double)> lLegAController;
		OperationCaller<double(double, double, double, double)> lLegBController;
		OperationCaller<double(double, double, double, double)> lLegHController;
		OperationCaller<double(double, double, double, double)> rLegAController;
		OperationCaller<double(double, double, double, double)> rLegBController;
		OperationCaller<double(double, double, double, double)> rLegHController;
		OperationCaller<void(double, double, double)>           lLegASmoothInit;
		OperationCaller<MotorState(void)>                       lLegASmoothController;
		OperationCaller<void(double, double, double)>           lLegBSmoothInit;
		OperationCaller<MotorState(void)>                       lLegBSmoothController;
		OperationCaller<void(double, double, double)>           lLegHSmoothInit;
		OperationCaller<MotorState(void)>                       lLegHSmoothController;
		OperationCaller<void(double, double, double)>           rLegASmoothInit;
		OperationCaller<MotorState(void)>                       rLegASmoothController;
		OperationCaller<void(double, double, double)>           rLegBSmoothInit;
		OperationCaller<MotorState(void)>                       rLegBSmoothController;
		OperationCaller<void(double, double, double)>           rLegHSmoothInit;
		OperationCaller<MotorState(void)>                       rLegHSmoothController;
		OperationCaller<MotorAngle(double, double)>             legToMotorPos;

		Property<double> lLegAP;
		Property<double> lLegAD;
		Property<double> lLegBP;
		Property<double> lLegBD;
		Property<double> lLegHP;
		Property<double> lLegHD;
		Property<double> rLegAP;
		Property<double> rLegAD;
		Property<double> rLegBP;
		Property<double> rLegBD;
		Property<double> rLegHP;
		Property<double> rLegHD;
		Property<bool>   lLegASmoothFinished;
		Property<bool>   lLegBSmoothFinished;
		Property<bool>   lLegHSmoothFinished;
		Property<bool>   rLegASmoothFinished;
		Property<bool>   rLegBSmoothFinished;
		Property<bool>   rLegHSmoothFinished;

		/** @brief This represents the "init" state. It's called periodically during smooth initialization.
		  * @return The controller output for this state.
		  */
		atrias_msgs::controller_output stateInit(atrias_msgs::robot_state& rs);
		void setStateInit();

		/** @brief This represents the "flight" state. This is called periodically.
		  * @return The desired robot state (angle for each motor).
		  */
		RobotPos stateFlight();
		void setStateFlight();

		// Our "locked leg" state
		atrias_msgs::controller_output stateLocked(atrias_msgs::robot_state& rs);
		void setStateLocked();

		void lLegFlightGains();
		void rLegFlightGains();

		// This Operation is called by the RT Operations Manager.
		atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);
		
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

// vim: noexpandtab
