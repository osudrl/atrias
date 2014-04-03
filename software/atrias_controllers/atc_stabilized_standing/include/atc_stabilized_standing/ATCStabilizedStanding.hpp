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
//static const double K[4][14] = {
//	{-1448.69515160330, -549.725670759006,  12.6917380239717, -6.28828347262183, -9.66894175108183, -20.1645698132088, -513.427612367629, -184.108591558064, -513.427612368862, -184.108591558327, -97.7052793395228, -32.3951045929388, -97.7052793418931, -32.3951045932228},
//	{-1448.69515160170, -549.725670758395, -9.66894175101718, -20.1645698131799,  12.6917380239267, -6.28828347260726, -513.427612367090, -184.108591557864, -513.427612368316, -184.108591558127, -97.7052793394009, -32.3951045929042, -97.7052793417639, -32.3951045931877},
//	{ 1754.08508317985,  658.588306284351,  11.0777107579311,  20.8527164066587,  11.0777107579194,  20.8527164066640,  695.869639411444,  244.549883660771,  498.486064519323,  202.035566139870,  371.633968918444,  62.4103846543252, -10.9790568858466,  17.0710988807653},
//	{ 1754.08508325619,  658.588306313498,  11.0777107584466,  20.8527164077100,  11.0777107584348,  20.8527164077157,  498.486064544060,  202.035566149211,  695.869639439136,  244.549883670755, -10.9790568826310,  17.0710988821002,  371.633968927323,  62.4103846563403} };

// Feasible fixed point states
//static const double XStar[14] = {1.57079632679490, 0, 3.93699148377394, 0, 2.34619382340565, 0, 3.53135338638727, 0, 2.75183192079232, 0, 3.46485251243322, 0, 2.81833279474637, 0};

// Feasible fixed point controls
//static const double UStar[4] = {0, 0, -106.401398326484, 106.401398326484};

// LQR gain matrix
static const double K[4][14] = {
	{-4316.0347385432214650791138410568, -1579.3932881845194060588255524635,  40.927012475551336478929442819208,  9.6223568940005321792341419495642, -29.783665643128642841475084424019, -47.499899235357290194770030211657, -1598.4068522810594004113227128983,  -551.3342150984306044847471639514, -1598.4068522794041200540959835052, -551.33421509810204952373169362545, -209.92881442173256800742819905281, -65.498501274197167276724940165877, -209.92881442003067604673560708761, -65.498501274029905516727012582123},
	{-4316.0347385407476394902914762497, -1579.3932881837190507212653756142, -29.783665643086933982885966543108, -47.499899235334268610131402965635,  40.927012475541459934902377426624,  9.6223568940229853296841611154377, -1598.4068522802590450737625360489, -551.33421509813956618017982691526, -1598.4068522785310051403939723969, -551.33421509781101121916435658932, -209.92881442157795390812680125237, -65.498501274163061225408455356956,  -209.9288144199033467884873971343, -65.498501273998073202164960093796},
	{ 1756.9198654244473800645209848881,  633.96017310165871094795875251293,  11.040636103859419847594836028293,  16.460122245224660275653150165454,  11.040636103872991213847853941843,   16.46012224522564082462849910371,  756.77126624536322196945548057556,   252.8762666681333257656660862267,  506.77173193485486990539357066154,  198.53209470310082451760536059737,  293.56974024348062357603339478374,  40.531931504607541683071758598089, -7.9837940024938234273577108979225,  12.428117952334602591690781991929},
	{ 1756.9198654018255183473229408264,  633.96017309337094047805294394493,  11.040636103705011805686808656901,  16.460122244976730598864378407598,  11.040636103713239890566910617054,  16.460122244977526406728429719806,   506.7717319273288012482225894928,  198.53209470034335026866756379604,  756.77126623666845262050628662109,  252.87626666511550865834578871727, -7.9837940031484322389587759971619,  12.428117952070039109457866288722,  293.56974024136559364706045016646,  40.531931504209950389849836938083} };

// Feasible fixed point states
static const double XStar[14] = {1.5707963267948966192313216916398, 0, 3.936991483773936817414096367429, 0, 2.3461938234056494145818305696594, 0, 3.5926194653860554772961677372223, 0, 2.6905658417935307546997591998661, 0, 3.5163311271867905283272648375714, 0, 2.766854179992795703668662099517, 0};

// Feasible fixed point controls
static const double UStar[4] = {0, 0, -122.06134111882400361537293065339, 122.06134111882400361537293065339};

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
