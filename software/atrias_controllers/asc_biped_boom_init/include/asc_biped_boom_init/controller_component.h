#ifndef __ASC_BIPED_BOOM_INIT_H__
#define __ASC_BIPED_BOOM_INIT_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for the asc_biped_boom_init subcontroller.
 */

// Orocos 
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/controller_structs.h>
#include <asc_biped_boom_init/controller_log_data.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_biped_boom_init;

namespace atrias {
namespace controller {

class ASCBipedBoomInit : public TaskContext {
private:
    // Operations
    void passRobotState(atrias_msgs::robot_state _rs);
    int isInitialized(void);
    MotorCurrent leftLeg(double aTargetPos, double bTargetPos);
    double capCurrent(double current);

    // Operation variables
    atrias_msgs::robot_state rs;
    int stateCheck;
    MotorCurrent motorCurrent;

    // Subcontroller names
    std::string pd0Name;

    // Subcontroller components
    TaskContext *pd0;

    // Service properties
    Property<double> D0;
    Property<double> P0;

    // Subcontroller operations
    OperationCaller<double(double, double, double, double)> pd0RunController;

    // Math variables
    double targetPos, currentPos, targetVel, currentVel;
    double cap;

    // Logging
    controller_log_data logData;
    OutputPort<controller_log_data> logPort;

public:
    // Constructor
    ASCBipedBoomInit(std::string name);

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
