#include <rtt/RTT.hpp>
#include <iostream>
#include <sys/time.h>

#include <rtt/os/main.h>

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>

#include "structs.h"

using namespace std;
using namespace RTT;
using namespace Orocos;

class OrocosProcessSpamWorker: public RTT::TaskContext {
    InputPort<orocosData> dataInPort;
    OutputPort<orocosData> dataOutPort;
    InputPort<orocosData> dummyInPort;
    OutputPort<orocosData> dummyOutPort;
    uint32_t usec1, usec2, usecLast, usecDiff1, usecDiff2;
    orocosData dataVar;

public:
    OrocosProcessSpamWorker(std::string name) :
        RTT::TaskContext(name),
            dataInPort("data_in"),
            dataOutPort("data_out"),
            dummyInPort("dummy_in"),
            dummyOutPort("dummy_out") {

        this->addEventPort(dataInPort);
        this->addPort(dataOutPort);
        this->addPort(dummyInPort);
        this->addPort(dummyOutPort);
    }

    bool configureHook() {
        dataOutPort.setDataSample(dataVar);
        dummyOutPort.setDataSample(dataVar);
        //std::cout << "Orocos_process_spam configured !" << std::endl;
        return true;
    }

    bool startHook() {
        //std::cout << "Orocos_process_spam started !" << std::endl;
        return true;
    }

    void updateHook() {
        dummyInPort.read(dataVar);
        dummyOutPort.write(dataVar);
        dataInPort.read(dataVar);
        dataOutPort.write(dataVar);
        //log(Info) << "Worker relaying data!" << endlog();
    }

    void stopHook() {
        // Unlock memory.
    }

    void cleanupHook() {
        //std::cout << "Orocos_process_spam cleaning up !" << std::endl;
    }
};
ORO_CREATE_COMPONENT(OrocosProcessSpamWorker);
