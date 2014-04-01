#ifndef ATCStabilizedStanding_HPP
#define ATCStabilizedStanding_HPP

/**
 * @file ATCStabilizedStanding.hpp
 * @author Mikhail S. Jones
 * @brief A standing controller for balancing on one leg using a Linear Quadratic Regulator.
 */

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>

// Our logging data type.
#include "atc_stabilized_standing/controller_log_data.h"

// The type transmitted from the GUI to the controller
#include "atc_stabilized_standing/controller_input.h"

// The type transmitted from the controller to the GUI
#include "atc_stabilized_standing/controller_status.h"

// Our subcontroller types
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>
#include <asc_pd/ASCPD.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>

// Datatypes
#include <robot_invariant_defs.h>
#include <robot_variant_defs.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

// LQR gain matrix
static const double K[4][14] = {
	{-1448.69515160330, -549.725670759006,  12.6917380239717, -6.28828347262183, -9.66894175108183, -20.1645698132088, -513.427612367629, -184.108591558064, -513.427612368862, -184.108591558327, -97.7052793395228, -32.3951045929388, -97.7052793418931, -32.3951045932228},
	{-1448.69515160170, -549.725670758395, -9.66894175101718, -20.1645698131799,  12.6917380239267, -6.28828347260726, -513.427612367090, -184.108591557864, -513.427612368316, -184.108591558127, -97.7052793394009, -32.3951045929042, -97.7052793417639, -32.3951045931877},
	{ 1754.08508317985,  658.588306284351,  11.0777107579311,  20.8527164066587,  11.0777107579194,  20.8527164066640,  695.869639411444,  244.549883660771,  498.486064519323,  202.035566139870,  371.633968918444,  62.4103846543252, -10.9790568858466,  17.0710988807653},
	{ 1754.08508325619,  658.588306313498,  11.0777107584466,  20.8527164077100,  11.0777107584348,  20.8527164077157,  498.486064544060,  202.035566149211,  695.869639439136,  244.549883670755, -10.9790568826310,  17.0710988821002,  371.633968927323,  62.4103846563403} };

// Feasible fixed point states
static const double XStar[14] = {1.57079632679490, 0, 3.93699148377394, 0, 2.34619382340565, 0, 3.53135338638727, 0, 2.75183192079232, 0, 3.46485251243322, 0, 2.81833279474637, 0};


// Feasible fixed point controls
static const double UStar[4] = {0, 0, -106.401398326484, 106.401398326484};

class ATCStabilizedStanding : public ATC<
	atc_stabilized_standing::controller_log_data_,
	atc_stabilized_standing::controller_input_,
	atc_stabilized_standing::controller_status_>
{
	public:
		/** 
		 * @brief The constructor for this controller.
		 * @param name The name of this component.
		 * Every top-level controller will have this name parameter,
		 * just like current controllers.
		 */
		ATCStabilizedStanding(string name);

	private:
		/** 
		 * @brief This is the main function for the top-level controller.
		 */
		void controller();

		/**
     * @brief These are functions for the top-level controller.
     */
    void updateController();
    void checkSafeties(); 
    void hipController();
    void startupController();
    void stabilizationController();
    void shutdownController();

    /**
     * @brief These are sub controllers used by the top level controller.
     */
    ASCHipBoomKinematics ascHipBoomKinematics;
    ASCPD ascPDLmA, ascPDLmB, ascPDRmA, ascPDRmB, ascPDLh, ascPDRh;
    ASCRateLimit ascRateLimitLh, ascRateLimitLmA, ascRateLimitLmB, ascRateLimitRh, ascRateLimitRmA, ascRateLimitRmB;

    /**
     * @brief These are all the variables used by the top level controller.
     */
    // Controller state variables
    int controllerState;

    // Hip state variables
    double qLh, qRh; // Hip states
    LeftRight toePosition; // Desired toe positions measures from boom center axis

    // Motor and leg variables
    double qmLA, qmLB, qmRA, qmRB; // Motor states

    // Misc margins, rate limiters and other debug values
    double legMotorRateLimit, hipMotorRateLimit;
    double currentLimit, deflectionLimit, velocityLimit;

    // For-loop iterators
    int i, j;

    // Control and state matrices
    double X[14], U[4];
};

}
}

#endif // ATCStabilizedStanding_HPP
