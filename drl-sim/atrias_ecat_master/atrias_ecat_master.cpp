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
//#include <sstream>

// ATRIAS
#include <atrias_ecat_master/uspace_controller_wrapper.h>
#include <atrias_msgs/atrias_controller_requests.h>   // Inputs from GUI.
#include <atrias_msgs/atrias_data.h>   // Data stream.
#include <atrias_msgs/atrias_debug.h>   // Debug stream.

using namespace std;
using namespace RTT;
using namespace Orocos;


class AtriasEthercatMaster : public TaskContext {
    InputPort<atrias_msgs::atrias_controller_requests>  crInPort;
    OutputPort<atrias_msgs::atrias_data>                dataOutPort;
    OutputPort<atrias_msgs::atrias_debug>               debugOutPort;
    std::string prop_answer;

int counter;

atrias_msgs::atrias_data ad;
atrias_msgs::atrias_controller_requests cr;
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
        crInPort        ("controller_requests_in"),
        dataOutPort     ("data_out"),
        debugOutPort    ("debug_out"),
        prop_answer     ("aem_prop_answer")
    {
        log(Info) << "AtriasEthercatMaster constructed !" <<endlog();

        this->addEventPort  (crInPort);
        this->addPort       (dataOutPort);
        this->addPort       (debugOutPort);
        this->addProperty   ("answer", prop_answer);

        // Add operation with flag ClientThread. See Section 3.4 of Orocos
        // Component Manual.
        //this->addOperation("publish", &AtriasEthercatMaster::publish, this, ClientThread).doc("Publish data?");

        //#ifdef BUILD_AS_EXECUTABLE
        //// Syntax: Activity (int scheduler, int priority, Seconds period, base::RunnableInterface *r=0, const std::string &name="Activity")
        //this->setActivity( new Activity(ORO_SCHED_RT, 99, 0.001, NULL, "aemAct") );
        //#endif // BUILD_AS_EXECUTABLE

        counter = 0;
    }

    bool configureHook() {
        log(Info) << "AEM configured !" <<endlog();
        init_master();

        #ifndef REAL_TIME
        usec1     = 0;
        usec2     = 0;
        usecLast  = 0;
        usecDiff1 = 0;
        usecDiff2 = 0;
        #endif // REAL_TIME

        return true;
    }

    bool startHook() {
        log(Info) << "AEM started !" <<endlog();
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
        if (NewData == crInPort.read(cr)) {
            //log(Info) << "[AEM] command: " << cr.command << endlog();
            //log(Info) << "[AEM] controller_requested: " << (int) cr.controller_requested << endlog();

            // Load controller request information into other control index...
            shm.controller_data[1 - shm.control_index].command = cr.command;
            shm.controller_data[1 - shm.control_index].controller_requested = cr.controller_requested;
            for (int i=0; i<SIZE_OF_CONTROLLER_DATA; i++) {   // TODO: SIZE_OF_CONTROLLER_DATA is 100, but this number is hardcoded into the ROS msg. This may need to change.
                shm.controller_data[1 - shm.control_index].data[i] = cr.control_data[i];
            }

            // ...then request the switch.
            shm.req_switch = true;
        }

        // Run the cyclic task of running the controller wrapper and
        // sending/receiving over EtherCAT.
        if (atrias_connected) {
            cyclic_task();
        }

        // Send data to ACS.
        ControllerInput* c_in = &shm.controller_input;
        ControllerOutput* c_out = &shm.controller_output;
        ControllerState* c_state = &shm.controller_state;

        ad.time = counter;
        ad.body_angle       = c_in->body_angle;
        ad.body_angle_vel   = c_in->body_angle_vel;
        ad.motor_angleA     = c_in->motor_angleA;
        ad.motor_angleA_inc = c_in->motor_angleA_inc;
        ad.motor_angleB     = c_in->motor_angleB;
        ad.motor_angleB_inc = c_in->motor_angleB_inc;
        ad.leg_angleA       = c_in->leg_angleA;
        ad.leg_angleB       = c_in->leg_angleB;

        ad.motor_velocityA = c_in->motor_velocityA;
        ad.motor_velocityB = c_in->motor_velocityB;
        ad.leg_velocityA   = c_in->leg_velocityA;
        ad.leg_velocityB   = c_in->leg_velocityB;

        ad.hip_angle     = c_in->hip_angle;
        ad.hip_angle_vel = c_in->hip_angle_vel;

        ad.xPosition = c_in->xPosition;
        ad.yPosition = c_in->yPosition;
        ad.zPosition = c_in->zPosition;

        ad.xVelocity = c_in->xVelocity;
        ad.yVelocity = c_in->yVelocity;
        ad.zVelocity = c_in->zVelocity;

        ad.motor_currentA = c_in->motor_currentA;
        ad.motor_currentB = c_in->motor_currentB;

        ad.toe_switch = c_in->toe_switch;

        ad.command = c_in->command;

        for (int i=0; i<3; i++) {
            ad.thermistorA[i] = c_in->thermistorA[i];
            ad.thermistorB[i] = c_in->thermistorB[i];
        }
        //ad.thermistorB[0] = 4.0;
        ad.motorVoltageA  = c_in->motorVoltageA;
        ad.motorVoltageB  = c_in->motorVoltageB;
        ad.logicVoltageA  = c_in->logicVoltageA;
        ad.logicVoltageB  = c_in->logicVoltageB;
        ad.medullaStatusA = c_in->medullaStatusA;
        ad.medullaStatusB = c_in->medullaStatusB;

        for (int i=0; i<SIZE_OF_CONTROLLER_STATE_DATA; i++) {
            ad.control_state[i] = c_state->data[i];
        }

        ad.motor_torqueA = c_out->motor_torqueA;
        ad.motor_torqueB = c_out->motor_torqueB;

        // Write to port.
        dataOutPort.write(ad);

        counter++;

        #ifndef REAL_TIME
        gettimeofday (&tv, NULL);
        tm = localtime (&tv.tv_sec);

        usec2     = tm->tm_sec*1000000 + tv.tv_usec;
        usecDiff2 = usec2 - usec1;

        char buffer[250];
        sprintf(buffer, "AEM updateHook!  Loop interval: %d,  time: %d", usecDiff1, usecDiff2);
        debugOut.string1 = buffer;
        debugOut.float1  = usecDiff1;
        debugOut.float2  = usecDiff2;
        debugOutPort.write(debugOut);
        #endif // REAL_TIME
    }

    void stopHook() {
        log(Info) << "AEM stopping !" <<endlog();
    }

    void cleanupHook() {
        log(Info) << "AEM cleaning up !" <<endlog();
    }
};
ORO_CREATE_COMPONENT(AtriasEthercatMaster)

