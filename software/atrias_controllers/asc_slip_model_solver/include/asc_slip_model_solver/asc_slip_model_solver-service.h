#ifndef __ASC_SLIP_MODEL_SOLVER_H__
#define __ASC_SLIP_MODEL_SOLVER_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace std;

namespace atrias {
namespace controller {

// ASCSlipModelSolver ==========================================================
class ASCSlipModelSolver : public Service {
    public:
        // Constructor
        ASCSlipModelSolver(TaskContext* owner);

        // Operations
        SlipConditions slipAdvanceTimeStep(SlipModel slipModel, SlipConditions slipConditions);
		LegForce slipConditionsToForce(SlipModel slipModel, SlipConditions slipConditions);

    private:
		double delta;
		LegForce legForce;

}; // class ASCSlipModelSolver

} // namespace controller
} // namespace atrias

#endif
