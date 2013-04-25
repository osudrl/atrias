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
#include <asc_pd/ASCPD.hpp>
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>

// Datatypes
#include <atrias_shared/controller_structs.h>

// Namespaces we're using
using namespace std;
using namespace atc_slip_walking;

// Our namespaces
namespace atrias {
namespace controller {

class ATCSlipWalking : public ATC<atc_slip_walking::controller_log_data, controller_input, controller_status> {
    public:
        ATCSlipWalking(string name);

    private:
        void controller();

        // Sub controllers
        ASCCommonToolkit commonToolkit;
        ASCPD pdHip;
        ASCPD pdStanceLeg;
        ASCPD pdFlightLeg;
        ASCHipBoomKinematics hipBoomKinematics;

        void guiCommunication();

        // Event Angles
        double qE;  // Extension
        double qTD; // TouchDown
        double qS1; // Enter single support
        double qS2; // Exit single support
        double qTO; // TakeOff

        // Hip control
        void hipControlSetup();
        void hipControl();
        double qlh, qrh;
        LeftRight toePosition;

        // Standing
        void standingControl();
        double qrl, qll, lrl, lll;      // SLIP model
        double qrmA, qrmB, qlmA, qlmB;  // ATRIAS motors

        // Walking
        void walkingControl();
        // eqPoint
        void eqPointWalkingSetup();
        void eqPointWalkingControl();
        atrias_msgs::robot_state_leg *rsStanceLeg, *rsFlightLeg;
        atrias_msgs::controller_output_leg *coStanceLeg, *coFlightLeg;
        // State switching and ground contact
        bool sw_stance, sw_flight, rGC, lGC;
        double t, s;
        // Angles
        double qsl, qfl, qfmA, qfmB, qsmA, qsmB, qsmB_des;
        // Lengths
        double amp, lfl, lsl;
        // SLIP
        void slipWalking();

        // Shutdown
        void shutdownControl();

};

}
}

#endif // ATCSlipWalking_HPP

// vim: expandtab
