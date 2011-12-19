#ifndef OROCOS_ATRIAS_CONTROLLER_WRAPPER_COMPONENT_HPP
#define OROCOS_ATRIAS_CONTROLLER_WRAPPER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>

class Atrias_controller_wrapper
    : public RTT::TaskContext
{
 public:
    Atrias_controller_wrapper(string const& name)
        : TaskContext(name)
    {
        std::cout << "Atrias_controller_wrapper constructed !" <<std::endl;
    }

    bool configureHook() {
        std::cout << "Atrias_controller_wrapper configured !" <<std::endl;
        return true;
    }

    bool startHook() {
        std::cout << "Atrias_controller_wrapper started !" <<std::endl;
        return true;
    }

    void updateHook() {
        std::cout << "Atrias_controller_wrapper executes updateHook !" <<std::endl;
    }

    void stopHook() {
        std::cout << "Atrias_controller_wrapper executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "Atrias_controller_wrapper cleaning up !" <<std::endl;
    }
};

#endif
