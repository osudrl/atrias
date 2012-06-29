/*! \file atrias_controller_wrapper.cpp
 *  \author Soo-Hyun Yoo
 *  \brief ATRIAS controller wrapper.
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
#include <std_msgs/String.h>
//#include <atrias_controller_wrapper/atrias_data.h>   // Data stream.
//#include <atrias_controller_wrapper/atrias_controller_requests.h>   // Inputs from GUI.

// C
#include <stdlib.h>

using namespace std;
using namespace RTT;
using namespace Orocos;


class AtriasControllerWrapper : public TaskContext {
    //InputPort<atrias_controller_wrapper::atrias_controller_requests> controllerRequestsInPort;
    //OutputPort<atrias_controller_wrapper::atrias_data> data50HzOutPort;
    //OutputPort<atrias_controller_wrapper::atrias_data> data1000HzOutPort;
    //std::string prop_answer;

public:
    AtriasControllerWrapper(std::string name):
        RTT::TaskContext(name)/*,
        controllerRequestsInPort("atrias_controller_requests"),
        data50HzOutPort("data_out_50_hz"),
        data1000HzOutPort("data_out_1000_hz"),
        prop_answer("acw_prop_answer")*/
    {
        log(Info) << "AtriasControllerWrapper constructed !" <<endlog();

        //this->addEventPort(controllerRequestsInPort);
        //this->addPort(data50HzOutPort);
        //this->addPort(data1000HzOutPort);
        //this->addProperty("answer", prop_answer);

        // Add operation with flag ClientThread. See Section 3.4 of Orocos
        // Component Manual.
        //this->addOperation("publish", &AtriasControllerWrapper::publish, this, ClientThread).doc("Publish data?");
    }

    bool configureHook() {
        log(Info) << "ACW configured !" <<endlog();
        return true;
    }

    bool startHook() {
        log(Info) << "ACW started !" <<endlog();
        return true;
    }

    void updateHook() {
        //atrias_controller_wrapper::atrias_controller_requests cr;

        //if (NewData == controllerRequestsInPort.read(cr)) {
            //log(Info) << "Controller requested: " << cr.controller_requested << endlog();
        //}
        //data1000HzOutPort.write(___);

        //ss << "ACW executes updateHook!  Time: " << usecDiff;
        log(Info) << "ACW updateHook!" << endlog();
    }

    void stopHook() {
        log(Info) << "ACW stopping !" <<endlog();
    }

    void cleanupHook() {
        log(Info) << "ACW cleaning up !" <<endlog();
    }
};
ORO_CREATE_COMPONENT(AtriasControllerWrapper)

