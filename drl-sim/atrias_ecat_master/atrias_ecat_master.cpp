/*! \file atrias_ecat_master.cpp
 *  \author Soo-Hyun Yoo
 *  \brief ATRIAS EtherCAT Master.
 *
 *  Details.
 */

//#define REAL_TIME

// Orocos
#include <rtt/os/main.h>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>

// ROS
#include <ros/ros.h>

// C
//#include <iostream>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/mman.h>
//#include <sstream>

// ATRIAS
#include <atrias_ecat_master/uspace_controller_wrapper.h>
#include <atrias_controller_wrapper/controller_wrapper_states.h>
#include <atrias_msgs/controller_input.h>   // Inputs from GUI.
#include <atrias_msgs/controller_status.h>   // Data stream.
#include <atrias_msgs/atrias_debug.h>   // Debug stream.

using namespace std;
using namespace RTT;
using namespace Orocos;


class AtriasEthercatMaster : public TaskContext {
    InputPort<atrias_msgs::controller_input>            ciInPort;
    OutputPort<atrias_msgs::controller_status>          csOutPort;
    OutputPort<atrias_msgs::atrias_debug>               debugOutPort;

    int counter;

    atrias_msgs::controller_status cs;
    atrias_msgs::controller_input ci;
    atrias_msgs::atrias_debug debugOut;

    #ifndef REAL_TIME
    int usec1, usec2, usecLast, usecDiff1, usecDiff2;

    // Get system time. NOTE: This counts as a system call, which is not
    // realtime-safe.. but whatever?
    struct timeval tv;
    struct tm *tm;
    #endif // REAL_TIME

public:
    AtriasEthercatMaster(std::string name):
        RTT::TaskContext(name),
        ciInPort        ("ci_in"),
        csOutPort       ("cs_out"),
        debugOutPort    ("debug_out")
    {
        log(Info) << "[AEM] constructed!" << endlog();

        this->addPort       (ciInPort);
        this->addPort       (csOutPort);
        this->addPort       (debugOutPort);

        //#ifdef BUILD_AS_EXECUTABLE
        //// Syntax: Activity (int scheduler, int priority, Seconds period, base::RunnableInterface *r=0, const std::string &name="Activity")
        //this->setActivity( new Activity(ORO_SCHED_RT, 99, 0.001, NULL, "aemAct") );
        //#endif // BUILD_AS_EXECUTABLE

        counter = 0;
    }

    bool configureHook() {
        log(Info) << "[AEM] configured!" << endlog();
        init_master();

        #ifndef REAL_TIME
        usec1     = 0;
        usec2     = 0;
        usecLast  = 0;
        usecDiff1 = 0;
        usecDiff2 = 0;
        #endif // REAL_TIME

        // Lock memory.
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            perror("mlockall");
        }

        // Show data sample to the port (this is supposed to be necessary for
        // real-time operation).
        csOutPort.setDataSample(cs);

        // Initialize bad timestamp counters.
        for (int i=0; i<NUM_OF_MEDULLAS_ON_ROBOT; i++) {
            medulla_bad_timestamp_counter[i] = 0;
        }

        return true;
    }

    bool startHook() {
        log(Info) << "[AEM] started!" << endlog();
        return true;
    }

    void updateHook() {
        #ifndef REAL_TIME
        gettimeofday (&tv, NULL);
        tm = localtime (&tv.tv_sec);

        usec1     = tm->tm_sec*1000000 + tv.tv_usec;
        usecDiff1 = usec1 - usecLast;
        usecLast  = usec1;
        #endif // REAL_TIME

        // Does ACS have a new controller request?
        if (NewData == ciInPort.read(ci)) {
            //log(Info) << "[AEM] command: " << cr.command << endlog();
            //log(Info) << "[AEM] controller_requested: " << (int) cr.controller_requested << endlog();

            // Load controller request information
            cwd.controllerStatus.state.command = ci.command;
            cwd.controllerInput = ci;
        }

        // Run the cyclic task of running the controller wrapper and
        // sending/receiving over EtherCAT.
        //if (atrias_connected) {
            cyclic_task();
        //}

        // Send data to ACS.
        cs = cwd.controllerStatus;

        // Write to port.
        csOutPort.write(cs);

        counter++;

        #ifndef REAL_TIME
        gettimeofday (&tv, NULL);
        tm = localtime (&tv.tv_sec);

        usec2     = tm->tm_sec*1000000 + tv.tv_usec;
        usecDiff2 = usec2 - usec1;

        /*char buffer[250];
        sprintf(buffer, "AEM updateHook!  Loop interval: %d,  time: %d", usecDiff1, usecDiff2);
        debugOut.string1 = buffer;
        debugOut.float1  = usecDiff1;
        debugOut.float2  = usecDiff2;
        debugOutPort.write(debugOut);*/
        cwd.controllerStatus.state.loopTime = usecDiff2;
        #endif // REAL_TIME
    }

    void stopHook() {
        // Unlock memory.
        if (munlockall() == -1) {
            perror("munlockall");
        }

        log(Info) << "[AEM] stopped!" << endlog();
    }

    void cleanupHook() {
        log(Info) << "[AEM] cleaned up!" << endlog();
    }
};
ORO_CREATE_COMPONENT(AtriasEthercatMaster)

