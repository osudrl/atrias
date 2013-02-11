#ifndef __ASC_LEG_FORCE_H__
#define __ASC_LEG_FORCE_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>

// Declare custom structures
struct MotorCurrent {
	double A;
	double B;
};
struct LegForce {
	double fx;
	double fz;
	double dfx;
	double dfz;
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
};

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
        MotorCurrent legForceToMotorCurrent(LegForce legForce, Leg leg, Position position);

    private:
    	double ks, kg, kp, kd;
		double l1, l2;
		double Fx, Fy, Fxdot, Fydot;
		//double qs1, qs2, qs3, qs4, qs9;
		//double qsdot1, qsdot2, qsdot3, qsdot4, qsdot9;
		double tauSpringA, tauSpringB, dtauSpringA, dtauSpringB;
		MotorCurrent motorCurrent;

}; // class ASCLegForce

} // namespace controller
} // namespace atrias

#endif
