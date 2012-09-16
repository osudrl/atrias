#include <asc_coordinate_transforms/leg_to_motor_transforms-service.h>

namespace atrias {
namespace controller {
// Constructor
LegToMotorTransforms::LegToMotorTransforms(TaskContext* owner)
    : Service("legToMotorTransforms", owner)
{
    this->addOperation("posTransform", &LegToMotorTransforms::posTransform, this)
        .doc("Given a leg angle and length, returns a struct of the corresponding motor angles.");
    this->addOperation("velTransform", &LegToMotorTransforms::velTransform, this)
        .doc("Given a leg angle and length, returns a struct of the corresponding motor angular velocities.");
}

MotorAngle LegToMotorTransforms::posTransform(double legAng, double legLen)
{
    motorAngle.A = legAng - acos(legLen);
    motorAngle.B = legAng + acos(legLen);
    return motorAngle;
}

MotorVelocity LegToMotorTransforms::velTransform(MotorState legAng, MotorState legLen)
{
    motorVelocity.A = legAng.vel + legLen.vel / sqrt(1.0 - legLen.ang*legLen.ang);
    motorVelocity.B = legAng.vel - legLen.vel / sqrt(1.0 - legLen.ang*legLen.ang);
    return motorVelocity;
}

ORO_SERVICE_NAMED_PLUGIN(LegToMotorTransforms, "legToMotorTransforms")

}
}
