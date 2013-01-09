#include "atrias_csim_conn/CSimConn.h"

namespace atrias {

namespace cSimConn {

CSimConn::CSimConn(std::string name) :
         RTT::TaskContext(name),
         newStateCallback("newStateCallback"),
         gazeboDataIn("gazebo_data_in"),
         gazeboDataOut("gazebo_data_out") {
	this->provides("connector")
	    ->addOperation("sendControllerOutput", &CSimConn::sendControllerOutput, this, RTT::ClientThread);
	this->requires("rtOps")
	    ->addOperationCaller(newStateCallback);
	
	addPort(gazeboDataIn);
	addPort(gazeboDataOut);
}

bool CSimConn::configureHook() {
	RTT::TaskContext *peer = this->getPeer("atrias_rt");
	if (!peer) {
		log(RTT::Error) << "[CSimConn] Failed to connect to RTOps!" << RTT::endlog();
		return false;
	}
	newStateCallback = peer->provides("rtOps")->getOperation("newStateCallback");
	log(RTT::Info) << "[CSimConn] Connected to RTOps." << RTT::endlog();
	log(RTT::Info) << "[CSimConn] configured!" << RTT::endlog();
	return true;
}

void CSimConn::sendControllerOutput(atrias_msgs::controller_output controller_output) {
	gazeboDataOut.write(controller_output);
	return;
}

void CSimConn::updateHook() {
	atrias_msgs::robot_state state;
	if (RTT::NewData == gazeboDataIn.read(state)) {
		newStateCallback(state);
	}
}

ORO_CREATE_COMPONENT(CSimConn)

}

}
