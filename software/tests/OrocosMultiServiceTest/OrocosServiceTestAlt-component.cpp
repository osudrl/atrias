#include "OrocosServiceTestAlt-component.hpp"

OrocosServiceTest::OrocosServiceTest(std::string const& name)
:
    TaskContext(name),
    setVar("setVar"),
    getVar("getVar")
{
    //bool b = this->requires("myservice")->addOperationCaller(getRandomNumber);
    //std::cout << "Success: " << (b ? "true" : "false") << std::endl;
    this->requires("myservice")->addOperationCaller(setVar);
    this->requires("myservice")->addOperationCaller(getVar);
    //std::cout << "OrocosServiceTest constructed !" <<std::endl;
}

uint64_t OrocosServiceTest::getMicroSecs() {
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    return ((uint64_t)tm->tm_sec)*1000000 + ((uint64_t)tv.tv_usec);
}

bool OrocosServiceTest::configureHook() {
    //std::cout << "OrocosServiceTest configured !" <<std::endl;
    //this->requires("myservice")->ready();
    return true;
}

bool OrocosServiceTest::startHook() {
    setVar.send((uint8_t)5);
    setVar
    return true;
}

void OrocosServiceTest::updateHook() {
}

void OrocosServiceTest::stopHook() {
    running = false;
    //std::cout << "OrocosServiceTest executes stopping !" <<std::endl;
}

void OrocosServiceTest::cleanupHook() {
    //std::cout << "OrocosServiceTest cleaning up !" <<std::endl;
}

ORO_CREATE_COMPONENT(OrocosServiceTest)
