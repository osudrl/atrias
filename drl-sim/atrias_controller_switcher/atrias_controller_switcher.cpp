/*! \file atrias_controller_switcher.cpp
 *  \author Soo-Hyun Yoo
 *  \brief ATRIAS controller switcher.
 *
 *  Details.
 */

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
#include <atrias_msgs/controller_input.h>   // Inputs from GUI.
#include <atrias_msgs/controller_status.h>   // Data stream.
#include <atrias_msgs/atrias_debug.h>   // Debug stream.

using namespace std;
using namespace RTT;
using namespace Orocos;

class AtriasControllerSwitcher : public TaskContext {
    // I/O with AEM.
    InputPort<atrias_msgs::controller_status>           csInPort;
    InputPort<atrias_msgs::atrias_debug>                debugInPort;
    OutputPort<atrias_msgs::controller_input>           ciOutPort;

    // I/O with ROS topics.
    InputPort<atrias_msgs::controller_input>            ciInPort;
    OutputPort<atrias_msgs::atrias_debug>               debugOutPort;
    OutputPort<atrias_msgs::controller_status>          cs50HzOutPort;
    OutputPort<atrias_msgs::controller_status>          cs1000HzOutPort;

    int counter;

    atrias_msgs::controller_input ci;
    atrias_msgs::controller_status cs;
    atrias_msgs::atrias_debug adebug;

public:
    AtriasControllerSwitcher(std::string name):
        RTT::TaskContext(name),
        csInPort            ("cs_in"),
        debugInPort         ("debug_in"),
        ciOutPort           ("ci_out"),
        ciInPort            ("ci_in"),
        debugOutPort        ("debug_out"),
        cs50HzOutPort       ("cs_50_hz_out"),
        cs1000HzOutPort     ("cs_1000_hz_out")
    {
        //log(Info) << "AtriasControllerSwitcher constructed !" <<endlog();

        this->addEventPort  (csInPort);
        this->addEventPort  (debugInPort);
        this->addPort       (ciOutPort);
        this->addEventPort  (ciInPort);
        this->addPort       (debugOutPort);
        this->addPort       (cs50HzOutPort);
        this->addPort       (cs1000HzOutPort);

        counter = 0;

        // Add operation with flag ClientThread. See Section 3.4 of Orocos
        // Component Manual.
        //this->addOperation("publish", &AtriasControllerSwitcher::publish, this, ClientThread).doc("Publish data?");
    }

    bool configureHook() {
        // Show ports what data sizes to expect to guarantee real-time transfer.
        ciOutPort.setDataSample(ci);

        log(Info) << "[ACS] configured!" << endlog();
        return true;
    }

    bool startHook() {
        log(Info) << "[ACS] started!" << endlog();
        return true;
    }

    void updateHook() {
        // Check for new controller requests from GUI.
        if (NewData == ciInPort.read(ci)) {
            //log(Info) << "[ACS] Controller requested: " << (int) cr.controller_requested << endlog();
            // Pass on the controller request to AEM.
            ciOutPort.write(ci);
        }

        // Check for new data from AEM.
        if (NewData == csInPort.read(cs)) {
            //log(Info) << "[ACS] Data received from AEM." << endlog();
            //log(Info) << "[ACS] ad.thermistorB[0]: " << ad.thermistorB[0] << endlog();

            // Write data to ROS topics.
            if (counter % 20 == 0) {   // 1000/20 = 50 Hz
                cs50HzOutPort.write(cs);
                //counter = 0;

                // Print state machine information.
                //log(Info) << "[ACS] Time: " << (int) ad.time << "  Counter: " << counter << "  Command: " << (int) ad.command << "  Controller requested: " << (int) cr.controller_requested << endlog();
            }
            cs1000HzOutPort.write(cs);
            counter = (counter+1) % 1000;
        }

        if (NewData == debugInPort.read(adebug)) {
            debugOutPort.write(adebug);
        }
        //log(Info) << "ACS updateHook!" << endlog();
    }

    void stopHook() {
        log(Info) << "[ACS] stopped!" << endlog();
    }

    void cleanupHook() {
        log(Info) << "[ACS] cleaned up!" << endlog();
    }
};
ORO_CREATE_COMPONENT(AtriasControllerSwitcher)

