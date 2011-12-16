#ifndef OROCOS_ATRIAS_OROCOS_TEST_COMPONENT_HPP
#define OROCOS_ATRIAS_OROCOS_TEST_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>

#include <sys/time.h>

class Atrias_orocos_test
    : public RTT::TaskContext
{

int count;

// Get system time. NOTE: This counts as system calls, which is not realtime-safe. But whatever?
struct timeval tv;
struct timezone tz;
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
        count = 0;
        return true;
    }

    void updateHook() {
        gettimeofday(&tv, &tz);
        tm = localtime(&tv.tv_sec);

        std::cout << "Atrias_orocos_test executes updateHook !   Time: " << tm->tm_sec*1000000 + tv.tv_usec - count << " Count: " << count <<std::endl;

        count = tm->tm_sec*1000000 + tv.tv_usec;
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
