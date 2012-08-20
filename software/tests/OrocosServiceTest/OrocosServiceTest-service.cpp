#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include "structs.h"

using namespace RTT;
using namespace std;

/**
 * An example service which can be loaded in a component.
 */
class MyService : public RTT::Service {
public:
    MyService(TaskContext* owner) 
        : Service("myservice", owner)
    {
        this->addOperation("getRandomNumber", &MyService::getRandomNumber, this);
    }

    crazyData getRandomNumber(crazyData cd) {
        cd.f[0] = 5.;
        return cd;
    }
};

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(MyService, "myservice")
