#ifndef OROCOS_ATRIAS_OROCOS_TEST_COMPONENT_HPP
#define OROCOS_ATRIAS_OROCOS_TEST_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>

#include <sys/time.h>

#include <soem_master/soem_driver.h>

class Atrias_orocos_test
    : public RTT::TaskContext
{

//int count;
int usecNow, usecLast, usecDiff;

// Get system time. NOTE: This counts as a system call, which is not
// realtime-safe.. but whatever?
struct timeval tv;
struct tm *tm;

 public:
    Atrias_orocos_test(string const& name)
        : TaskContext(name)
    {
        std::cout << "Atrias_orocos_test constructed !" <<std::endl;
    }

    bool configureHook() {
        std::cout << "Atrias_orocos_test configured !" <<std::endl;
        return true;
    }

    bool startHook() {
        std::cout << "Atrias_orocos_test started !" <<std::endl;
        //count = 0;
        usecNow = 0;
        usecLast = 0;
        usecDiff = 0;
        return true;
    }

    void updateHook() {
        gettimeofday(&tv, NULL);
        tm = localtime(&tv.tv_sec);

        usecNow = tm->tm_sec*1000000 + tv.tv_usec;
        usecDiff = usecNow - usecLast;
        usecLast = usecNow;

        std::cout << "Atrias_orocos_test executes updateHook !   Time: " << usecDiff <<std::endl;   //<< " Count: " << count <<std::endl;

        //count++;
    }

    void stopHook() {
        std::cout << "Atrias_orocos_test executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "Atrias_orocos_test cleaning up !" <<std::endl;
    }
};

#endif
