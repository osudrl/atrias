#include "atrias_control_lib/ATC.hpp"

namespace atrias {
namespace controller {

template <typename logType, typename guiInType, typename guiOutType>
ATC<logType, guiInType, guiOutType>::ATC(const std::string& name) :
	RTT::TaskContext(name),
	AtriasController(name)
{
	// The magic is done above
}

template <typename logType, typename guiInType, typename guiOutType>
RTT::TaskContext& ATC<logType, guiInType, guiOutType>::getTaskContext() const {
	return *this;
}

}
}

// vim: noexpandtab
