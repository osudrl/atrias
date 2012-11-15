#ifndef __ASC_FORCE_DEFL_H__
#define __ASC_FORCE_DEFL_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for the asc_force_defl subcontroller.
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
#include <asc_force_defl/controller_log_data.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_force_defl;

namespace atrias {
namespace controller {

class ASCForceDefl : public TaskContext {
private:
    // Operations
    double runController(double exampleInput);

    double out;

    // Subcontroller names
    std::string torqueDefl0Name;

    // Subcontroller components
    TaskContext *torqueDefl0;

    // Service properties
    Property<std::string> linearInterp0Name0;

    // Subcontroller operations
    OperationCaller<double(double)> torqueDefl0GetDefl;

    // Logging
    controller_log_data logData;
    OutputPort<controller_log_data> logPort;

public:
    // Constructor
    ASCForceDefl(std::string name);

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
