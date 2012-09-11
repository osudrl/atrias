#ifndef __ASC_PD_H__
#define __ASC_PD_H__

/*! \file controller_component.h
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component header for the PD subcontroller.
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
#include <asc_pd/controller_log_data.h>

using namespace RTT;
using namespace Orocos;

namespace atrias {
namespace controller {

class ASCPD : public TaskContext {
private:
    // This Operation is called by the RT Operations Manager.
    double runController(double targetPos, double currentPos, double targetVel, double currentVel);

    double P, D, out;

    // For logging
    asc_pd::controller_log_data logData;
    OutputPort<asc_pd::controller_log_data> logPort;

public:
    // Constructor
    ASCPD(std::string name);

    /** @brief Get ROS header from RTOps.
     */
    RTT::OperationCaller<std_msgs::Header(void)> getROSHeader;

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
