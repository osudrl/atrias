/*! \file ATCMotorSinWave.hpp
 *  \author Andrew Peekema
 *  \brief Header for leg sin wave controller.
 */

#ifndef ATCMotorSinWave_HPP
#define ATCMotorSinWave_HPP

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>

// Our logging data type
#include "atc_motor_sin_wave/controller_log_data.h"

// The type transmitted from the GUI to the controller
#include "atc_motor_sin_wave/controller_input.h"

// The type transmitted from the controller to the GUI
#include "atc_motor_sin_wave/controller_status.h"

// Our subcontroller types
#include <asc_pd/ASCPD.hpp>

// Datatypes
//#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>
#include <robot_variant_defs.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/controller_structs.h>

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

class ATCMotorSinWave : public ATC<
    atc_motor_sin_wave::controller_log_data_,
    atc_motor_sin_wave::controller_input_,
    atc_motor_sin_wave::controller_status_>
{
    public:
        /**
         * @brief The constructor for this controller.
         * @param name The name of this component.
         */
        ATCMotorSinWave(string name);

    private:
        /**
         * @brief This is the main function for the top-level controller.
         */
        void controller();

        /**
         * @brief These are sub controllers used by the top level controller.
         */
         ASCPD pd0Controller, pd1Controller, pd2Controller, pd3Controller;

        // Math variables
        double legP, legD;
        double centerAAngle, centerBAngle;
        double q, dq, t;
        double targetPos, currentPos, targetVel, currentVel;
}; // Class ATCMotorSinWave

} // namespace controller
} // namespace atrias

#endif // ATCMotorSinWave
