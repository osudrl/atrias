#ifndef __ASC_SIN_PATH_GENERATOR_H__
#define __ASC_SIN_PATH_GENERATOR_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for the sin generator subcontroller.
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
#include <asc_path_generator/controller_log_data.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;

namespace atrias {
namespace controller {

class ASCSinPathGenerator : public TaskContext {
private:
    // Operation
    MotorState runController(double frequency, double amplitude);
    void reset(void);

    MotorState sinOut;

    double accumulator;

    asc_path_generator::controller_log_data logData;
    OutputPort<asc_path_generator::controller_log_data> logPort;

public:
    // Constructor
    ASCSinPathGenerator(std::string name);

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
