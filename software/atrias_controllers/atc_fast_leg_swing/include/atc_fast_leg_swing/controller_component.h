#ifndef __ATC_FAST_LEG_SWING_H__
#define __ATC_FAST_LEG_SWING_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_fast_leg_swing controller.
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
#include <atc_fast_leg_swing/controller_input.h>
#include <atc_fast_leg_swing/controller_status.h>
#include <atc_fast_leg_swing/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_fast_leg_swing;

namespace atrias {
using namespace shared;
namespace controller {

class ATCFastLegSwing : public TaskContext {
	private:
		// This Operation is called by the RT Operations Manager.
		atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

		atrias_msgs::controller_output co;
		
		double lHipStart;
		double rHipStart;

        bool demoRunning;

		// Subcontroller names
        std::string spg0Name;
        std::string spg1Name;
        std::string spg2Name;
        std::string spg3Name;
        std::string spg4Name;
        std::string spg5Name;
		std::string pathGenerator0Name;
		std::string pathGenerator1Name;
		std::string pathGenerator2Name;
		std::string pathGenerator3Name;
		std::string pathGenerator4Name;
		std::string pathGenerator5Name;
		std::string pd0Name;
		std::string pd1Name;
		std::string pd2Name;
		std::string pd3Name;
		std::string pd4Name;
		std::string pd5Name;
		
		// Subcontroller components
        TaskContext *spg0;
        TaskContext *spg1;
        TaskContext *spg2;
        TaskContext *spg3;
        TaskContext *spg4;
        TaskContext *spg5;
		TaskContext *pathGenerator0;
		TaskContext *pathGenerator1;
		TaskContext *pathGenerator2;
		TaskContext *pathGenerator3;
		TaskContext *pathGenerator4;
		TaskContext *pathGenerator5;
		TaskContext *pd0;
		TaskContext *pd1;
		TaskContext *pd2;
		TaskContext *pd3;
		TaskContext *pd4;
		TaskContext *pd5;

		// Service properties
		Property<double> P0;
		Property<double> D0;
		Property<double> P1;
		Property<double> D1;
		Property<double> P2;
		Property<double> D2;
		Property<double> P3;
		Property<double> D3;
		Property<double> P4;
		Property<double> D4;
		Property<double> P5;
		Property<double> D5;
        Property<bool> spg0IsFinished;
        Property<bool> spg1IsFinished;
        Property<bool> spg2IsFinished;
        Property<bool> spg3IsFinished;
        Property<bool> spg4IsFinished;
        Property<bool> spg5IsFinished;
		
		// Subcontroller operations
        OperationCaller<void(double, double, double)> spg0Init;
        OperationCaller<void(double, double, double)> spg1Init;
        OperationCaller<void(double, double, double)> spg2Init;
        OperationCaller<void(double, double, double)> spg3Init;
        OperationCaller<void(double, double, double)> spg4Init;
        OperationCaller<void(double, double, double)> spg5Init;
        OperationCaller<MotorState(void)> spg0RunController;
        OperationCaller<MotorState(void)> spg1RunController;
        OperationCaller<MotorState(void)> spg2RunController;
        OperationCaller<MotorState(void)> spg3RunController;
        OperationCaller<MotorState(void)> spg4RunController;
        OperationCaller<MotorState(void)> spg5RunController;
		OperationCaller<MotorState(double, double)> path0Controller;
		OperationCaller<void(void)>                 path0ControllerReset;
		OperationCaller<MotorState(double, double)> path1Controller;
		OperationCaller<void(double)>               path1ControllerSetPhase;
		OperationCaller<void(void)>                 path1ControllerReset;
		OperationCaller<MotorState(double, double)> path2Controller;
		OperationCaller<void(double)>               path2ControllerSetPhase;
		OperationCaller<void(void)>                 path2ControllerReset;
		OperationCaller<MotorState(double, double)> path3Controller;
		OperationCaller<void(double)>               path3ControllerSetPhase;
		OperationCaller<void(void)>                 path3ControllerReset;
		OperationCaller<MotorState(double, double)> path4Controller;
		OperationCaller<void(double)>               path4ControllerSetPhase;
		OperationCaller<void(void)>                 path4ControllerReset;
		OperationCaller<MotorState(double, double)> path5Controller;
		OperationCaller<void(double)>               path5ControllerSetPhase;
		OperationCaller<void(void)>                 path5ControllerReset;
		OperationCaller<double(double, double, double, double)> pd0Controller;
		OperationCaller<double(double, double, double, double)> pd1Controller;
		OperationCaller<double(double, double, double, double)> pd2Controller;
		OperationCaller<double(double, double, double, double)> pd3Controller;
		OperationCaller<double(double, double, double, double)> pd4Controller;
		OperationCaller<double(double, double, double, double)> pd5Controller;
		
		// Logging
		controller_log_data logData;
		OutputPort<controller_log_data>  logPort;

		// For the GUI
		shared::GuiPublishTimer                         *pubTimer;
		controller_input guiIn;
		InputPort<controller_input>                     guiDataIn;
		
	public:
		// Constructor
		ATCFastLegSwing(std::string name);

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
