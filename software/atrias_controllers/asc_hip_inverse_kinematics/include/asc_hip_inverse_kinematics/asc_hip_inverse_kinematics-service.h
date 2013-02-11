#ifndef __ASC_HIP_INVERSE_KINEMATICS_H__
#define __ASC_HIP_INVERSE_KINEMATICS_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>
#include <atrias_msgs/robot_state.h>
#include <complex.h>

using namespace RTT;
using namespace std;

namespace atrias {
namespace controller {

// ASCHipInverseKinematics =====================================================
class ASCHipInverseKinematics : public Service {
    public:
        // Constructor
        ASCHipInverseKinematics(TaskContext* owner);

        // Operations
        LeftRight toePositionToHipAngle(LeftRight toePosition, atrias_msgs::robot_state_leg lLeg, atrias_msgs::robot_state_leg rLeg, atrias_msgs::robot_state_location position);

    private:
    	complex<double> i;
    	complex<double> complexHipAngleLeft;
    	complex<double> complexHipAngleRight;
		double l1, l2;
		double lBoom, lBody, lHip, qBodyOffset;
		double lLeftLeg, lRightLeg, qLeftLeg, qRightLeg;
        LeftRight hipAngle;

}; // class ASCHipInverseKinematics

} // namespace controller
} // namespace atrias

#endif
