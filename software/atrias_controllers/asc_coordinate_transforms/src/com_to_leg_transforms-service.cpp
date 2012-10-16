#include <asc_coordinate_transforms/com_to_leg_transforms-service.h>

namespace atrias {
namespace controller {
// Constructor
ComToLegTransforms::ComToLegTransforms(TaskContext* owner)
    : Service("comToLegTransforms", owner)
{
    this->addOperation("transforms", &ComToLegTransforms::transform, this)
        .doc("Given the center of mass x, z, dx, dz, and step length, returns the desired state of the robot");
}

DesiredRobotState ComToLegTransforms::transform(double x, double z, double dx, double dz, double stepLength)
{
    // Assumptions:
    //      The right leg points to x=0
    //      The left leg points to x=stepLength
    //      -stepLength < x < stepLength
    //      It may work over more x, but the math hasn't been checked

    // Shorthand for calculations
    xl  = stepLength - x;
    dxl = -dx;
    xr  = x;
    dxr = dx;

    // Leg Lengths
    ds.left.leg.len  = pow(xl*xl + z*z, 0.5);
    ds.right.leg.len = pow(xr*xr + z*z, 0.5);
    // Leg Angles
    ds.left.leg.ang  = atan(z/xl);
    ds.right.leg.ang = M_PI/2.0 + atan(xr/z);
    // Leg Length Change
    ds.left.leg.lenVel  = (xl*dxl + z*dz)/pow(xl*xl + z*z, 0.5);
    ds.right.leg.lenVel = (xr*dxr + z*dz)/pow(xr*xr + z*z, 0.5);
    // Leg Angle Change
    ds.left.leg.angVel  = (xl*dz - z*dxl)/(xl*xl + z*z);
    ds.right.leg.angVel = (z*dxr - xr*dz)/(z*z + xr*xr);

    // No Hip Calculations
    ds.left.hip.ang  = 0.0;
    ds.left.hip.vel  = 0.0;
    ds.right.hip.ang = 0.0;
    ds.right.hip.vel = 0.0;

    return ds;
}

ORO_SERVICE_NAMED_PLUGIN(ComToLegTransforms, "comToLegTransforms")

}
}
