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


class OrocosProcessSpam: public RTT::TaskContext {
    InputPort<orocosData> dataInPort;
    OutputPort<orocosData> dataOutPort;
    InputPort<orocosData> dummyInPort;
    OutputPort<orocosData> dummyOutPort;
    orocosData dataVar;
    struct timeval tv;
    struct tm *tm;
    vector<int16_t> *times;

public:
    OrocosProcessSpam(std::string name) :
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

    uint64_t getMicroSecs() {
        gettimeofday(&tv, NULL);
        tm = localtime(&tv.tv_sec);
        return ((uint64_t)tm->tm_sec)*1000000 + ((uint64_t)tv.tv_usec);
    }

    bool configureHook() {
        dataOutPort.setDataSample(dataVar);
        dummyOutPort.setDataSample(dataVar);
        times = new vector<int16_t>();
        std::cout << "Orocos_process_spam configured !" << std::endl;
        return true;
    }

    bool startHook() {
        dataVar.time = getMicroSecs();
        dataOutPort.write(dataVar);
        std::cout << "Orocos_process_spam started !" << std::endl;
        return true;
    }

    void updateHook() {
        dummyInPort.read(dataVar);
        dataInPort.read(dataVar);
        __suseconds_t currentuSecs = getMicroSecs();
        int16_t time = (int16_t)(currentuSecs - dataVar.time);
        times->push_back(time);
        //log(Info) << "Elapsed time: " << time << " microseconds" << endlog();
        //dataVar.time = currentuSecs;
        dataVar.time = getMicroSecs();
        dummyOutPort.write(dataVar);
        dataOutPort.write(dataVar);
    }

    void stopHook() {
        uint64_t total = 0.;
        size_t len = times->size();
        for (size_t i = 0; i < len; i++)
            total += (*times)[i];
        total /= len;
        log(Info) << "Average time: " << total << " microseconds" << endlog();
        // Unlock memory.
        /*if (munlockall() == -1) {
            perror("munlockall");
        }*/
    }

    void cleanupHook() {
        std::cout << "Orocos_process_spam cleaning up !" << std::endl;
    }
};
ORO_CREATE_COMPONENT(OrocosProcessSpam);
