#include "ToSubstitutePackageName/ToSubstituteClassName.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ToSubstituteClassName::ToSubstituteClassName(string name) :
     ATC(name)
{
     // Do init here.
}

void ToSubstituteClassName::controller() {
	// Implement your controller here. The robot state is in rs -- put the controller
	// output in co
}

ORO_CREATE_COMPONENT(ToSubstituteClassName)

}
}
