#ifndef __ASC_BIPED_BOOM_INIT_H__
#define __ASC_BIPED_BOOM_INIT_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for the asc_init_biped_boom subcontroller.
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
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/globals.h>
#include <asc_init_biped_boom/controller_log_data.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_init_biped_boom;

namespace atrias {
namespace controller {

class ASCInitBipedBoom : public TaskContext {
private:
    // Operations
    bool done(void);
    bool init(atrias_msgs::robot_state rs, RobotPos pos);
    atrias_msgs::controller_output run(atrias_msgs::robot_state rs);

    // Operation variables
    atrias_msgs::controller_output co;
    uint8_t robotConfig;

    // Subcontroller names
    std::string ASCPD0Name;
    std::string smoothPath0Name;
    std::string smoothPath1Name;
    std::string smoothPath2Name;
    std::string smoothPath3Name;
    std::string smoothPath4Name;
    std::string smoothPath5Name;

    // Subcontroller components
    TaskContext *pd0;
    TaskContext *smoothPath0;
    TaskContext *smoothPath1;
    TaskContext *smoothPath2;
    TaskContext *smoothPath3;
    TaskContext *smoothPath4;
    TaskContext *smoothPath5;

    // Service properties
    Property<double> D0;
    Property<double> P0;

    // Subcontroller operations
    OperationCaller<double(double, double, double, double)> pd0RunController;
    OperationCaller<void(double, double)> smoothPath0Init;
    OperationCaller<void(double, double)> smoothPath1Init;
    OperationCaller<void(double, double)> smoothPath2Init;
    OperationCaller<void(double, double)> smoothPath3Init;
    OperationCaller<void(double, double)> smoothPath4Init;
    OperationCaller<void(double, double)> smoothPath5Init;
    OperationCaller<MotorState(double, double)> smoothPath0Run;
    OperationCaller<MotorState(double, double)> smoothPath1Run;
    OperationCaller<MotorState(double, double)> smoothPath2Run;
    OperationCaller<MotorState(double, double)> smoothPath3Run;
    OperationCaller<MotorState(double, double)> smoothPath4Run;
    OperationCaller<MotorState(double, double)> smoothPath5Run;

    // Math variables
    double targetPos, currentPos, targetVel, currentVel;
    MotorState lLegAState, lLegBState;
    double frequency, amplitude;

    // Logging
    controller_log_data logData;
    OutputPort<controller_log_data> logPort;

public:
    // Constructor
    ASCInitBipedBoom(std::string name);

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
