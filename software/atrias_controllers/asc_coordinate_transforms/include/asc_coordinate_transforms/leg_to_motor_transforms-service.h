#ifndef __LEG_TO_MOTOR_TRANSFORMS_SERVICE_H__
#define __LEG_TO_MOTOR_TRANSFORMS_SERVICE_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace std;

namespace atrias {
namespace controller {

class LegToMotorTransforms : public Service {
    public:
        // Constructor
        LegToMotorTransforms(TaskContext* owner);

        // Operations
        MotorAngle posTransform(double legAng, double legLen);
        MotorVelocity velTransform(MotorState legAng, MotorState legLen);

    private:
        MotorAngle motorAngle;
        MotorVelocity motorVelocity;
};
}
}

#endif
