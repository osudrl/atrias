#include "atc_deadbeat_control/ATCDeadbeatControl.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ATCDeadbeatControl::ATCDeadbeatControl(string name) :
     ATC(name)
{
     // Do init here.
}

void ATCDeadbeatControl::controller() {
	// Implement your controller here. The robot state is in rs -- put the controller
	// output in co
}

ORO_CREATE_COMPONENT(ATCDeadbeatControl)

}
}
