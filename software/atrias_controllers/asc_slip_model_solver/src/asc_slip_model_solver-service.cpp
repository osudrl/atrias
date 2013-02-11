#include <asc_slip_model_solver/asc_slip_model_solver-service.h>

namespace atrias {
namespace controller {

// Constructor
ASCSlipModelSolver::ASCSlipModelSolver(TaskContext* owner)
    : Service("exampleService", owner)
{
    this->addOperation("exampleOperation", &ASCSlipModelSolver::exampleOperation, this)
        .doc("This operation multiplies its two arguments and returns that value.");
}

double ASCSlipModelSolver::exampleOperation(double arg1, double arg2)
{
    out = arg1*arg2;
    return out;
}

ORO_SERVICE_NAMED_PLUGIN(ASCSlipModelSolver, "exampleService")

}
}
