#include "atc_motor_position/ATCMP.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCMP::ATCMP(string name) :
     ATC(name),
     pdLA(this, "example_subcontroller")
{
     // We don't need to do much here, just call the ATC() constructor above.
}

void ATCMP::controller() {
	// Sample logging data
     logOut.lATgt = rateLimLA(guiIn.des_motor_ang_left_A,    rate);

     // Set gains
     pdLA.P = pdLB.P = pdRA.P = pdRB.P = guiIn.leg_motor_p_gain;

     // Command the outputs (and copy to our logging data).
     co.lLeg.motorCurrentA = pdLA(logOut.lATgt, rs.lLeg.halfA.rotorAngle, 0, rs.lLeg.halfA.rotorVelocity);

     /* Copy over positions to the GUI output data
      * Suppose that the guiOut message has the following members:
      *     float64 posA
      *     float64 posB
      */
     guiOut.isEnabled             = isEnabled();

     /* Additionally, the following functions are available to command the robot state:
      * commandHalt();    // Trigger a Halt
      * commandEStop();   // Trigger an EStop
      */
}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCMP)

}
}
