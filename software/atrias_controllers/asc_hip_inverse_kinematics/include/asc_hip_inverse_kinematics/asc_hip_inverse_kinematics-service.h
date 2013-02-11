#ifndef __ASC_HIP_INVERSE_KINEMATICS_H__
#define __ASC_HIP_INVERSE_KINEMATICS_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>
#include <complex.h>

struct ToePosition {
	double left;
	double right;
};
struct HipAngle {
	double left;
	double right;
};
struct LegHalf {
    double legAngle;
    double legVelocity;
	double motorAngle;
	double motorVelocity;
};
struct Leg {
	LegHalf halfA;
	LegHalf halfB;
};
struct Position {
	double bodyPitch;
	double bodyPitchVelocity;
	double boomAngle;
};

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
        HipAngle toePositionToHipAngle(ToePosition toePosition, Leg lLeg, Leg rLeg, Position position);

    private:
    	complex<double> i;
    	complex<double> complexHipAngleLeft;
    	complex<double> complexHipAngleRight;
		double l1, l2;
		double lBoom, lBody, lHip, qBodyOffset;
		double lLeftLeg, lRightLeg, qLeftLeg, qRightLeg;
        HipAngle hipAngle;

}; // class ASCHipInverseKinematics

} // namespace controller
} // namespace atrias

#endif
