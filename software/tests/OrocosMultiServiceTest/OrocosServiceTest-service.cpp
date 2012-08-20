#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include "structs.h"

using namespace RTT;
using namespace std;

/**
 * An example service which can be loaded in a component.
 */
class MyService : public RTT::Service {
private:
    uint8_t var;

public:
    MyService(TaskContext* owner) 
        : Service("myservice", owner)
    {
        this->addOperation("setVar", &MyService::getRandomNumber, this);
    }

    void setVar(uint8_t newVar) {
        var = newVar;
    }

    uint8_t getVar() {
        return var;
    }
};

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(MyService, "myservice")
