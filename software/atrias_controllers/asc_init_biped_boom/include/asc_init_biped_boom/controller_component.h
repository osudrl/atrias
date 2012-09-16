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
    atrias_msgs::controller_output run(atrias_msgs::robot_state rs, RobotPos pos);

    // Operation variables
    atrias_msgs::controller_output co;
    uint8_t robotConfig;

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
