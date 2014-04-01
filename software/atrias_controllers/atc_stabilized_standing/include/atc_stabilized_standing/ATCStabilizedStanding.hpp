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
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
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
	{-1317.82849152552, -496.394013653681,	4.01975128738442,	-12.9856618479765,	-5.98024871265565,	-17.6356089938057,	-450.858113233813,	-163.740114172286,	-450.858113233592,	-163.740114172232,	-101.886277475951,	-29.9493303026779,	-101.886277475608,	-29.9493303026334},
	{-1317.82849152281, -496.394013652655,	-5.98024871260154,	-17.6356089937590,	4.01975128735703,	-12.9856618479515,	-450.858113232889,	-163.740114171950,	-450.858113232675,	-163.740114171893,	-101.886277475729,	-29.9493303026165,	-101.886277475381,	-29.9493303025711},
	{1403.09323329453,	519.759215325270,	4.90296975989393,	13.7411085091392,	4.90296975995401,	13.7411085091539,	588.818990231253,	201.232460931421,	370.263311585529,	150.174977674528,	318.405103580821,	54.0001690714113,	-20.4712103095005,	12.1177386731425},
	{1403.09323328510,	519.759215321703,	4.90296975984779,	13.7411085090133,	4.90296975990933,	13.7411085090286,	370.263311582571,	150.174977673410,	588.818990227812,	201.232460930198,	-20.4712103099090,	12.1177386729716,	318.405103579689,	54.0001690711552} };

// Feasible fixed point states
static const double XStar[14] = {
	1.57079632679490,	0,	3.69261946538606,	0,	2.59056584179353,	0,	3.59261946538606,	0,	2.69056584179353,	0,	3.51633112718679,	0,	2.76685417999280,	0};

// Feasible fixed point controls
static const double UStar[4] = {
	0,	0,	-122.061341118824,	122.061341118824};

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
    ASCCommonToolkit ascCommonToolkit;
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

    // Iterators
    int i, j;

    // Control and state matrices
    double X[14], U[4];
};

}
}

#endif // ATCStabilizedStanding_HPP
