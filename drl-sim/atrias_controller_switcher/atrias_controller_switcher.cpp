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
#include <atrias_msgs/atrias_controller_requests.h>   // Inputs from GUI.
#include <atrias_msgs/atrias_data.h>   // Data stream.
#include <atrias_msgs/atrias_debug.h>   // Debug stream.

using namespace std;
using namespace RTT;
using namespace Orocos;

class AtriasControllerSwitcher : public TaskContext {
    // I/O with AEM.
    InputPort<atrias_msgs::atrias_data>                 dataInPort;
    InputPort<atrias_msgs::atrias_debug>                debugInPort;
    OutputPort<atrias_msgs::atrias_controller_requests> crOutPort;

    // I/O with ROS topics.
    InputPort<atrias_msgs::atrias_controller_requests>  crInPort;
    OutputPort<atrias_msgs::atrias_debug>               debugOutPort;
    OutputPort<atrias_msgs::atrias_data>                data50HzOutPort;
    OutputPort<atrias_msgs::atrias_data>                data1000HzOutPort;

    std::string prop_answer;

    int counter;

public:
    AtriasControllerSwitcher(std::string name):
        RTT::TaskContext(name),
        dataInPort          ("data_in"),
        debugInPort         ("debug_in"),
        crOutPort           ("controller_requests_out"),
        crInPort            ("controller_requests_in"),
        debugOutPort        ("debug_out"),
        data50HzOutPort     ("data_50_hz_out"),
        data1000HzOutPort   ("data_1000_hz_out"),
        prop_answer         ("acw_prop_answer")
    {
        log(Info) << "AtriasControllerSwitcher constructed !" <<endlog();

        this->addEventPort  (dataInPort);
        this->addEventPort  (debugInPort);
        this->addPort       (crOutPort);
        this->addEventPort  (crInPort);
        this->addPort       (debugOutPort);
        this->addPort       (data50HzOutPort);
        this->addPort       (data1000HzOutPort);
        this->addProperty   ("answer", prop_answer);

        counter = 0;

        // Add operation with flag ClientThread. See Section 3.4 of Orocos
        // Component Manual.
        //this->addOperation("publish", &AtriasControllerSwitcher::publish, this, ClientThread).doc("Publish data?");
    }

    bool configureHook() {
        log(Info) << "ACS configured !" <<endlog();
        return true;
    }

    bool startHook() {
        log(Info) << "ACS started !" <<endlog();
        return true;
    }

    void updateHook() {
        atrias_msgs::atrias_controller_requests cr;
        atrias_msgs::atrias_data ad;
        atrias_msgs::atrias_debug adebug;

        // Check for new controller requests from GUI.
        if (NewData == crInPort.read(cr)) {
            //log(Info) << "[ACS] Controller requested: " << (int) cr.controller_requested << endlog();
            // Pass on the controller request to AEM.
            crOutPort.write(cr);
        }

        // Check for new data from AEM.
        if (NewData == dataInPort.read(ad)) {
            //log(Info) << "[ACS] Data received from AEM." << endlog();
            //log(Info) << "[ACS] ad.thermistorB[0]: " << ad.thermistorB[0] << endlog();

            // Write data to ROS topics.
            if (counter % 20 == 0) {   // 1000/20 = 50 Hz
                data50HzOutPort.write(ad);
                //counter = 0;

                // Print state machine information.
                //log(Info) << "[ACS] Time: " << (int) ad.time << "  Counter: " << counter << "  Command: " << (int) ad.command << "  Controller requested: " << (int) cr.controller_requested << endlog();
            }
            data1000HzOutPort.write(ad);
            counter++;
        }

        if (NewData == debugInPort.read(adebug)) {
            debugOutPort.write(adebug);
        }
        //log(Info) << "ACS updateHook!" << endlog();
    }

    void stopHook() {
        log(Info) << "ACS stopping !" <<endlog();
    }

    void cleanupHook() {
        log(Info) << "ACS cleaning up !" <<endlog();
    }
};
ORO_CREATE_COMPONENT(AtriasControllerSwitcher)

