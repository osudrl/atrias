#ifndef ELABSCONN_H
#define ELABSCONN_H

#include <ecrt.h>
#include <ec_rtdm.h>
#include <rtdm/rtdm.h>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <time.h>
#include <errno.h>
#include <rtt/os/MutexLock.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/os/TimeService.hpp>

#include <robot_invariant_defs.h>

#include "atrias_elabs_conn/LegMedullaDefs.h"
#include "atrias_elabs_conn/LegMedulla.h"

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>

#define VENDOR_ID        MEDULLA_VENDOR_ID
#define PRODUCT_CODE0    MEDULLA_LEG_PRODUCT_CODE
#define PRODUCT_CODE1    MEDULLA_LEG_PRODUCT_CODE
#define ELABS_OP_STATE   1<<3
#define RT_DEV_FILE      "ec_rtdm0"
#define LOOP_PERIOD_NS   1000000
#define LOOP_OFFSET_NS   100000
#define EC_NEWTIMEVAL2NANO(TV) \
	(((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)
#define EC_NSECS_TO_NANO(NSECS) \
	((NSECS) - 946684800000000000LL)

using namespace RTT;
using namespace atrias::medullas;
namespace atrias {
namespace eLabsConn {
class ELabsConn : public TaskContext {
	/** @brief By calling this, we cycle RT Ops.
	  */
	RTT::OperationCaller<void(atrias_msgs::robot_state)>
		newStateCallback;
	
	/** @brief Holds the robot state to be passed to RT Ops.
	  */
	atrias_msgs::robot_state robotState;
	
	LegMedulla* lLegA;
	LegMedulla* lLegB;
	
	ec_master_t* ec_master;
	ec_domain_t* domain;
	int rt_fd;
	int counter;
	CstructMstrAttach MstrAttach;
	bool inOp;
	
	RTT::os::Mutex eCatLock;
	LEG_MEDULLA_OFFSETS(0);
	LEG_MEDULLA_OFFSETS(1);
	public:
		ELabsConn(std::string name);
		
		/** @brief Called by RT Ops w/ update controller torques.
		  * @param controller_output The new controller output.
		  */
		void sendControllerOutput(atrias_msgs::controller_output controller_output);
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};
}
}

#endif // ELABSCONN_H
