#ifndef ATCSlipWalking_HPP
#define ATCSlipWalking_HPP

/**
  * @file ATCSlipWalking.hpp
  * @author Ryan Van Why, modified by Andrew Peekema
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
#include <asc_pd/ASCPD.hpp>

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

        // PD controllers
        ASCPD pdLeg;
        ASCPD pdHip;
};

}
}

#endif // ATCSlipWalking_HPP

// vim: expandtab
