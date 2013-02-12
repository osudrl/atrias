#ifndef __ASC_LEG_FORCE_H__
#define __ASC_LEG_FORCE_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>
#include <atrias_msgs/robot_state.h>

using namespace RTT;
using namespace std;

namespace atrias {
namespace controller {

// ASCLegForce =================================================================
class ASCLegForce:public Service {
    public:
        // Constructor
        ASCLegForce(TaskContext* owner);

        // Operations
        AB legForceToMotorCurrent(LegForce legForce, Gain gain, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position);

    private:
		double l1, l2;
		double tauSpringA, tauSpringB;
		double dtauSpringA, dtauSpringB;
		AB motorCurrent;

}; // class ASCLegForce

} // namespace controller
} // namespace atrias

#endif
