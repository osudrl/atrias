#ifndef OROCOS_OROCOSSERVICETEST_COMPONENT_HPP
#define OROCOS_OROCOSSERVICETEST_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>
#include <sys/time.h>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include "structs.h"
#include <pthread.h>

class OrocosServiceTest
: public RTT::TaskContext
{
private:
    uint64_t counter;
    uint64_t timeStart;
    struct timeval tv;
    struct tm *tm;
    RTT::OperationCaller<crazyData(crazyData)> getRandomNumber;
    crazyData cd;
    bool running;

public:
    OrocosServiceTest(std::string const& name);
    uint64_t getMicroSecs();
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};

#endif
