#ifndef ATCSlipWalking_HPP
#define ATCSlipWalking_HPP

/**
  * @file ATCSlipWalking.hpp
  * @author Outline by Ryan Van Why, control code by Andrew Peekema
  * @brief A spring loaded inverted pendulum based walking controller
  */

// Top-level controllers are components, so we need to include this.
#include <rtt/Component.hpp>

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

// Our logging data type.
#include "atc_slip_walking/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "atc_slip_walking/controller_input.h"
// The type transmitted from the controller to the GUI
#include "atc_slip_walking/controller_status.h"

// Our subcontroller types
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
#include <asc_slip_model/ASCSlipModel.hpp>
#include <asc_leg_force/ASCLegForce.hpp>
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>
#include <asc_pd/ASCPD.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>

// Datatypes
#include <atrias_shared/controller_structs.h>

// Namespaces we're using
using namespace std;
using namespace atc_slip_walking;

// Our namespaces
namespace atrias {
namespace controller {

class ATCSlipWalking : public ATC<atc_slip_walking::controller_log_data_, controller_input_, controller_status_> {
    public:
        ATCSlipWalking(string name);

    private:
        void controller();

        // Subcontrollers
        ASCCommonToolkit commonToolkit;
        ASCSlipModel ascSlipModel;
        ASCLegForce ascLegForceLl;
        ASCLegForce ascLegForceRl;
        ASCHipBoomKinematics hipBoomKinematics;
        ASCPD pdLmA;
        ASCPD pdLmB;
        ASCPD pdRmA;
        ASCPD pdRmB;
        ASCPD pdLh;
        ASCPD pdRh;
        ASCRateLimit ascRateLimitLmA;
        ASCRateLimit ascRateLimitLmB;
        ASCRateLimit ascRateLimitRmA;
        ASCRateLimit ascRateLimitRmB;

        void guiCommunication();

        // Event Angles
        double qE;  // Extension
        double qTD; // TouchDown
        double qS1; // Enter single support
        double qS2; // Exit single support
        double qTO; // TakeOff

        // Motor and leg
        double legRateLimit;
        double qRl, qLl, rRl, rLl;      // SLIP model
        double qRmA, qRmB, qLmA, qLmB;  // ATRIAS motors

        // Hip control
        void hipControlSetup();
        void hipControl();
        double qlh, qrh;
        LeftRight toePosition;

        // Standing
        void standingControl();

        // Walking
        void walkingControl();

        // eqPoint
        void eqPointWalkingSetup();
        void eqPointWalkingControl();
        void eqPointStanceControl(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCPD*, ASCPD*);
        void eqPointFlightControl(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCPD*, ASCPD*);
        // State switching and ground contact
        uint8_t eqPointState;
        bool stanceComplete, flightComplete, rGC, lGC;
        double t, s;
        // Angles
        double qSl, qFl, qFmA, qFmB, qSmA, qSmB, qSmB_des;
        // Lengths
        double amp, rFl, rSl;

        // SLIP
        void slipWalkingSetup();
        void slipWalking();
        void virtualSpringStanceControl(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCLegForce*);
        void slipFlightControl(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCPD*, ASCPD*);
        uint8_t slipControlState;
        SlipState slipState;
        LegForce legForce;

        // Shutdown
        void shutdownControl();

};

}
}

#endif // ATCSlipWalking_HPP

// vim: expandtab
