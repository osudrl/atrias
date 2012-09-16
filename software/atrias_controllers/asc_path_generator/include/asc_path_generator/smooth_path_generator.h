#ifndef __ASC_SMOOTH_PATH_GENERATOR_H__
#define __ASC_SMOOTH_PATH_GENERATOR_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for the smooth path generator subcontroller.
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

class ASCSmoothPathGenerator : public TaskContext {
private:
    // Operation
    MotorState run(double frequency, double amplitude);
    void init(double startAng, double endAng);

    MotorState output;

    double accumulator;

    asc_path_generator::controller_log_data logData;
    OutputPort<asc_path_generator::controller_log_data> logPort;

public:
    // Constructor
    ASCSmoothPathGenerator(std::string name);

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
