#ifndef __ASC_COMPONENT_H__
#define __ASC_COMPONENT_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for the asc_component subcontroller.
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
#include <asc_component/controller_log_data.h>

using namespace RTT;
using namespace Orocos;

namespace atrias {
namespace controller {

class ASCComponent : public TaskContext {
private:
    // Operations
    double runController(double exampleInput);

    double out;

    // Subcontroller names

    // Subcontroller components

    // Service properties

    // Subcontroller operations

    // Logging
    asc_component::controller_log_data logData;
    OutputPort<asc_component::controller_log_data> logPort;

public:
    // Constructor
    ASCComponent(std::string name);

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
