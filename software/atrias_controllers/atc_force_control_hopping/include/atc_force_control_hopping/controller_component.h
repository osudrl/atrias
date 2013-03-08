#ifndef __ATC_FORCE_CONTROL_HOPPING_H__
#define __ATC_FORCE_CONTROL_HOPPING_H__

/*! \file controller_component.h
 *  \author Mikhail Jones
 *  \brief Orocos Component header for atc_force_control_hopping controller.
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
#include <atc_force_control_hopping/controller_input.h>
#include <atc_force_control_hopping/controller_status.h>
#include <atc_force_control_hopping/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_force_control_hopping;

namespace atrias {
using namespace shared;
namespace controller {

class ATCForceControlHopping : public TaskContext {

	private:
		// Operations ----------------------------------------------------------
		// This Operation is called by the RT Operations Manager.
		atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

		atrias_msgs::controller_output co;

		// Logging
		controller_log_data              logData;
		OutputPort<controller_log_data>  logPort;

		// For the GUI
		shared::GuiPublishTimer                         *pubTimer;
		controller_input                                guiIn;
		controller_status                               guiOut;
		OutputPort<controller_status>                   guiDataOut;
		InputPort<controller_input>                     guiDataIn;
		
		// ASCLegToMotorTransforms
		OperationCaller<MotorAngle(double, double)> legToMotorPos;
		
		// ASCHipBoomKinematics
		std::string ascHipBoomKinematics0Name;
		TaskContext *ascHipBoomKinematics0;
		OperationCaller<LeftRight(LeftRight, atrias_msgs::robot_state_leg, atrias_msgs::robot_state_leg, atrias_msgs::robot_state_location)> inverseKinematics0;
		
		// ASCLegForceControl
		std::string ascLegForceControl0Name;
		TaskContext *ascLegForceControl0;
		OperationCaller<AB(LegForce, Gain, atrias_msgs::robot_state_leg, atrias_msgs::robot_state_location)> legForceToMotorCurrent0;

		// ASCSlipModel
		std::string ascSlipModel0Name;
		TaskContext *ascSlipModel0;
		OperationCaller<SlipState(SlipModel, SlipState)> slipAdvance0;
		OperationCaller<LegForce(SlipModel, SlipState)> slipForce0;
	
		// Variables -----------------------------------------------------------
		bool isStance;
		bool isLeftStance;
		bool isRightStance;
		bool isInitialize, isDeinitialize, isFinished;// MARKED
		double t, duration;// MARKED
		
		// Leg position control variables
		MotorAngle lMotorAngle;
		MotorAngle rMotorAngle;
		double leftLegLen, leftLegAng, rightLegLen, rightLegAng;
	
		// Leg force control variables
		Gain gain;
		SlipState slipState;
		SlipModel slipModel;
		LegForce legForce;
		AB motorCurrent;
	
		// Hip control variables
		LeftRight toePosition;
		LeftRight hipAngle;

	public:
		// Constructor ---------------------------------------------------------
		ATCForceControlHopping(std::string name);
		
		// Log controller data -------------------------------------------------
	    RTT::OperationCaller<std_msgs::Header(void)> getROSHeader;

		// Standard Orocos hooks -----------------------------------------------
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		
}; // class ATCForceControlHopping

} // namespace controller
} // namespace atrias

#endif
