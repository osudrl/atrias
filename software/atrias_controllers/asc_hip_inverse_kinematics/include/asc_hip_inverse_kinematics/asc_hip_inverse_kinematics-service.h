#ifndef __ASC_HIP_INVERSE_KINEMATICS_H__
#define __ASC_HIP_INVERSE_KINEMATICS_H__

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace std;

namespace atrias {
namespace controller {

class ASCHipInverseKinematics : public Service {
    public:
        // Constructor
        ASCHipInverseKinematics(TaskContext* owner);

        // Operations
        double exampleOperation(double arg1, double arg2);

    private:
        double out;

};
}
}

#endif
