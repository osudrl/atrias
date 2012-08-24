#include <asc_service_plugin/asc_service_plugin-service.h>

namespace atrias {
namespace controller {

// Constructor
ASCServicePlugin::ASCServicePlugin(TaskContext* owner)
    : Service("exampleService", owner)
{
    this->addOperation("exampleOperation", &ASCServicePlugin::exampleOperation, this)
        .doc("This operation multiplies its two arguments and returns that value.");
}

double ASCServicePlugin::exampleOperation(double arg1, double arg2)
{
    out = arg1*arg2;
    return out;
}

ORO_SERVICE_NAMED_PLUGIN(ASCServicePlugin, "exampleService")

}
}
