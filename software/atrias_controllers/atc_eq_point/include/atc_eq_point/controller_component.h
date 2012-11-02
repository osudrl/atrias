#ifndef __ATC_EQ_POINT_H__
#define __ATC_EQ_POINT_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_eq_point controller.
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
#include <atc_eq_point/controller_input.h>
#include <atc_eq_point/controller_status.h>
#include <atc_eq_point/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_eq_point;

namespace atrias {
using namespace shared;
namespace controller {

class ATCEqPoint : public TaskContext {
private:
    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

    atrias_msgs::controller_output co;

	// Subcontroller names
	std::string spg0Name;
	std::string spg1Name;
	std::string spg2Name;
	std::string spg3Name;
	std::string spg4Name;
	std::string spg5Name;

	// Subcontroller components
	TaskContext *spg0;
	TaskContext *spg1;
	TaskContext *spg2;
	TaskContext *spg3;
	TaskContext *spg4;
	TaskContext *spg5;

	// Service properties
	Property<bool> spg0IsFinished;
	Property<bool> spg1IsFinished;
	Property<bool> spg2IsFinished;
	Property<bool> spg3IsFinished;
	Property<bool> spg4IsFinished;
	Property<bool> spg5IsFinished;

	// Subcontroller operations
	OperationCaller<MotorState(void)> spg0RunController;
	OperationCaller<MotorState(void)> spg1RunController;
	OperationCaller<MotorState(void)> spg2RunController;
	OperationCaller<MotorState(void)> spg3RunController;
	OperationCaller<MotorState(void)> spg4RunController;
	OperationCaller<MotorState(void)> spg5RunController;

    // Logging
    controller_log_data              logData;
    OutputPort<controller_log_data>  logPort;

    // For the GUI
    shared::GuiPublishTimer                         *pubTimer;
    controller_input                                guiIn;
    InputPort<controller_input>                     guiDataIn;

public:
    // Constructor
    ATCEqPoint(std::string name);

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
