#ifndef __COM_TO_LEG_TRANSFORMS_SERVICE_H__
#define __COM_TO_LEG_TRANSFORMS_SERVICE_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>
#include <math.h>

using namespace RTT;
using namespace std;

namespace atrias {
namespace controller {

class ComToLegTransforms : public Service {
    public:
        // Constructor
        ComToLegTransforms(TaskContext* owner);

        // Operations
        DesiredRobotState transform(double x, double z, double dx, double dz, double stepLength);

    private:
        DesiredRobotState ds;
        double xl, dxl, xr, dxr;
};
}
}

#endif
