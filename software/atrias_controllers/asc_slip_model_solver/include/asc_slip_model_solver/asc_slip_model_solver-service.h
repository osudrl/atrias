#ifndef __ASC_SLIP_MODEL_SOLVER_H__
#define __ASC_SLIP_MODEL_SOLVER_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace std;

namespace atrias {
namespace controller {

class ASCSlipModelSolver : public Service {
    public:
        // Constructor
        ASCSlipModelSolver(TaskContext* owner);

        // Operations
        double exampleOperation(double arg1, double arg2);

    private:
        double out;

};
}
}

#endif
