#ifndef __ASC_SLIP_MODEL_SOLVER_H__
#define __ASC_SLIP_MODEL_SOLVER_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>

// TODO - Move custom structures to controller structs.h

// Declare custom structures
struct SlipConditions {
	double rOld;
	double drOld;
	double qOld;
	double dqOld;
	double r;
	double dr;
	double q;
	double dq;
};
struct LegForce {
	double fx;
	double fz;
	double dfx;
	double dfz;
};

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
        SlipConditions slipAdvanceTimeStep(SlipConditions slipConditions);
		LegForce slipConditionsToForce(SlipConditions slipConditions);

    private:
		double delta;
		double g, ks, m, r0;
		LegForce legForce;

}; // class ASCSlipModelSolver

} // namespace controller
} // namespace atrias

#endif
