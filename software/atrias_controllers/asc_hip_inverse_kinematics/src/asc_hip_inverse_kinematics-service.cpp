#include <asc_hip_inverse_kinematics/asc_hip_inverse_kinematics-service.h>

namespace atrias {
namespace controller {

// Constructor
ASCHipInverseKinematics::ASCHipInverseKinematics(TaskContext* owner)
    : Service("exampleService", owner)
{
    this->addOperation("exampleOperation", &ASCHipInverseKinematics::exampleOperation, this)
        .doc("This operation multiplies its two arguments and returns that value.");
}

double ASCHipInverseKinematics::exampleOperation(double arg1, double arg2)
{
    out = arg1*arg2;
    return out;
}

ORO_SERVICE_NAMED_PLUGIN(ASCHipInverseKinematics, "exampleService")

}
}
