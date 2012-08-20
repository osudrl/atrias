#include <iostream>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OutputPort.hpp>

#include <atrias_rt_ops/dummy_type.h>

using namespace atrias_rt_ops;
using namespace std;
using namespace RTT;
using namespace Orocos;

class DeployerAffinitySetter: public RTT::TaskContext {
private:
    OutputPort<dummy_type> dummyPort;

public:
    DeployerAffinitySetter(std::string name) :
        TaskContext(name),
        dummyPort("dummyOutput") {
        addPort(dummyPort);
    }

    bool configureHook() {
        pid_t PID = fork();
        if (PID == 0) { // Child process
            int execResult = execlp("rosrun", "rosrun", "atrias", "set_deployer_realtime.sh", NULL);
            if (execResult < 0) {
                log(Warning) << "[AffinitySetter] Failed to set deployer to root cset!" << endlog();
            }
            exit(127);
        }
        else {
            waitpid(PID, 0, 0);
        }
        return true;
    }

    bool startHook() {
        return false;
    }

    void updateHook() {
    }

    void stopHook() {
    }

    void cleanupHook() {
    }
};
ORO_CREATE_COMPONENT(DeployerAffinitySetter);
