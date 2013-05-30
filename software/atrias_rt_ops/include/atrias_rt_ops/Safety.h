#ifndef SAFETY_H
#define SAFETY_H

/** @file
  * @brief Handles the safety features in RT Ops
  */

class Safety;

#include <robot_invariant_defs.h>
#include <atrias_msgs/robot_state.h>

#include "atrias_rt_ops/RTOps.h"

namespace atrias {

namespace rtOps {

class Safety {
	/** @brief Lets us access members of RT Ops.
	  */
	RTOps* rtOps;

	/** @brief Predicts where a motor will stop if we halt now.
	  * @param pos The motor's position
	  * @param vel The motor's velocity
	  * @return Its predicted stopping point.
	  */
	double predictStop(double pos, double vel);

	/**
	  * @brief This checks for a collision given motor stopping position
	  * @param lLegAPred The predicted stop location for left A motor
	  * @param lLegBPred The predicted stop location for left B motor
	  * @param rLegAPred The predicted stop location for right A motor
	  * @param rLegBPred The predicted stop location for right B motor
	  * @return true if there's a collision, false otherwise.
	  * This function will also send the correct event to report the collision detected
	  */
	bool checkCollision(double lLegAPred, double lLegBPred, double rLegAPred, double rLegBPred);

	public:
		/** @brief Initializes this Safety.
		  * @param rt_ops A pointer to RT Ops.
		  */
		Safety(RTOps* rt_ops);
		
		/** @brief Does the halt safety check.
		  * @return Whether or not the robot should halt.
		  */
		bool shouldHalt();
};

}

}

#endif // SAFETY_H
