#include "OrocosServiceTest-component.hpp"

OrocosServiceTest::OrocosServiceTest(std::string const& name)
:
    TaskContext(name),
    getRandomNumber("getRandomNumber")
{
    //bool b = this->requires("myservice")->addOperationCaller(getRandomNumber);
    //std::cout << "Success: " << (b ? "true" : "false") << std::endl;
    this->requires("myservice")->addOperationCaller(getRandomNumber);
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
    counter = 0;
    running = true;

    PeerList peers = getPeerList();
    for (size_t t = 0; t < peers.size(); t++) {
        std::cout << peers[t] << std::endl;
    }
    std::cout << "Total number of peers: " << peers.size() << std::endl;
    return true;
}

void OrocosServiceTest::updateHook() {
    timeStart = getMicroSecs();
    while(counter < 100000) {
        getRandomNumber.send(cd);
        counter++;
    }
    uint64_t diff = getMicroSecs() - timeStart;
    std::cout << "Performed " << counter << " calls in " << diff << " microseconds" << std::endl;
    std::cout << "Average of  " << ((double)diff / (double)counter) << " microseconds per call" << std::endl;
}

void OrocosServiceTest::stopHook() {
    running = false;
    //std::cout << "OrocosServiceTest executes stopping !" <<std::endl;
}

void OrocosServiceTest::cleanupHook() {
    //std::cout << "OrocosServiceTest cleaning up !" <<std::endl;
}

ORO_CREATE_COMPONENT(OrocosServiceTest)
