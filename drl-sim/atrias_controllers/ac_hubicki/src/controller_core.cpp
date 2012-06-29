/*
 * controller_core.cpp
 *
 * Hubicki Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <ac_hubicki/controller_core.h>

ControllerInitResult controllerInit() {
    in_flight = true;
    after_mid_stance = false;

    peak_ht = 1.0;

    time_of_last_stance = eclapsedTime = 0.;

	ControllerInitResult cir;
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = sizeof(ControllerStatus);
    cir.error = false;
    return cir;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);
    ControllerStatus cs;

    if (in_flight)
    {
        hubicki_flight_controller(&state, id, &cs, output);
    }
    else
    {
        hubicki_stance_controller(&state, id, &cs, output);
    }


    // Regardless of if we are in stance or flight we control the hip the same
    // Do that now.
    float des_hip_ang = 0.99366 * state.body_angle + 0.03705;

    // REMOVED BY HUBICKI!  Resulted in oscillation ourside of range
       // if ((des_hip_ang < -0.2007) || (des_hip_ang > 0.148))
       //        des_hip_ang = state->body_angle;

    //  Added Hubicki
    des_hip_ang = CLAMP(des_hip_ang, -0.2007, 0.148);
    // End Hubicki


    output->motor_torque_hip = id->stance_hip_p_gain * (des_hip_ang - state.motor_angle_hip)
                - id->stance_hip_d_gain * state.motor_velocity_hip;

    last_leg_len = cos((2. * PI + state.leg_angleA - state.leg_angleB) / 2.);

    cs.in_flight = in_flight;
    structToByteArray(cs, status);
}

void hubicki_flight_controller(robot_state* state, InputData* id, ControllerStatus* cs, ControllerOutput* output)
{

    // Spring deflections for force control.  These can be problematic on the real robot, since they require good sensor calibration.
    ///float spring_defA = state->leg_angleA - state->motor_angleA;
    //float spring_defB = state->leg_angleB - state->motor_angleB;

    // Negated 02/09
    float des_leg_ang = PI/2. - id->leg_ang_gain * state->xVelocity + id->hor_vel_gain * id->des_hor_vel;

    // Generate the motor torques.
    float des_mtr_angA = des_leg_ang - PI + acos(id->preferred_leg_len);
    float des_mtr_angB = des_leg_ang + PI - acos(id->preferred_leg_len);

    //float leg_angle = ( state->leg_angleA + state->leg_angleB ) / 2.;
    //float leg_length = - 0.5 * sin( state->leg_angleA ) - 0.5 * sin( state->leg_angleB );

    // XXX: This is a hack to keep the robot in one place.
    // GCF = gain control factor
    float gcf = CLAMP(MAX(ABS(des_mtr_angA - state->motor_angleA), ABS(des_mtr_angB - state->motor_angleB)) / 0.05, 0, 1);

    output->motor_torqueA = gcf * id->flight_p_gain * (des_mtr_angA - state->motor_angleA)
        - id->flight_d_gain * state->motor_velocityA;
    output->motor_torqueB = gcf * id->flight_p_gain * (des_mtr_angB - state->motor_angleB)
        - id->flight_d_gain * state->motor_velocityB;

    //=========================================================================//

    // Figure out the next state.
        //PRINT_MSG("00<%f> <%f>", ABS(spring_defA), ABS(spring_defB));
    //if ( ( state->zPosition - leg_length * sin( leg_angle ) < 0.02 ) && ( (ABS(state->motor_angleA - state->leg_angleA) > HUBICKI_CONTROLLER_DATA(data)->stance_spring_threshold)
    //  || (ABS(state->motor_angleB - state->leg_angleB) > HUBICKI_CONTROLLER_DATA(data)->stance_spring_threshold) ) )
    if ( state->toe_switch == 1)
    {
        // Check to see if ground contact has occured.
        //PRINT_MSG("TD!\n");

        in_flight = false;
    }

    // Check to see if we have reached a new peak height.
    peak_ht = MAX( state->zPosition, peak_ht );
}

void hubicki_stance_controller(robot_state* state, InputData* id, ControllerStatus* cs, ControllerOutput* output)
{
    // Spring deflections for force control.  These can be problematic on the real robot, since they require good sensor calibration.
    //float spring_defA = state->leg_angleA - state->motor_angleA;
    //float spring_defB = state->leg_angleB - state->motor_angleB;

    //float spring_def_velA = state->leg_velocityA - state->motor_velocityA;
    //float spring_def_velB = state->leg_velocityB - state->motor_velocityB;

    // Limit the desired leg length to help prevent smacking hardstops.
    float leg_len           = cos( ( 2.*PI + state->leg_angleA - state->leg_angleB ) / 2. );
    float zf_leg_len    = cos( ( 2.*PI + state->motor_angleA - state->motor_angleB ) / 2. ); // zero force leg length
    float zf_leg_len_vel = -sin( ( 2.*PI + state->motor_angleA - state->motor_angleB ) / 4.
        * ( state->motor_velocityA - state->motor_velocityB ) );

    // Check to see if the robot has reached midstance.  If it has, set the after mid stance flag.
    if ( ( !after_mid_stance ) && ( leg_len > last_leg_len ) )
    {
        after_mid_stance = true;
    }

//HUBICKI DEBUG ADD
//HUBICKI_CONTROLLER_STATE(state)->after_mid_stance = false;
// END DEBUG!


    // Find the leg extension during stance to add energy back into the system.
    float leg_ext = 0.;

    // If the robot has reach midstance, extend the leg.
    if ( after_mid_stance )
    {
        leg_ext = id->hop_ht_gain * ( id->des_hop_ht - peak_ht );
    }

    // Limit the desired leg length.
    float des_leg_len = CLAMP( id->preferred_leg_len + leg_ext, 0.51, 0.97 );
    float torque = id->stance_p_gain * (des_leg_len - zf_leg_len )
        - id->stance_d_gain * zf_leg_len_vel
        + ESTIMATED_SPRING_STIFFNESS * (zf_leg_len - leg_len) / ESTIMATED_GEAR_RATIO;

    //float leg_angle = ( state->leg_angleA + state->leg_angleB ) / 2.;
    //float leg_length = - 0.5 * sin( state->leg_angleA ) - 0.5 * sin( state->leg_angleB );

    output->motor_torqueA =  -torque;
    output->motor_torqueB =  torque;

    //float des_leg_ang = (state->leg_angleA + state->leg_angleB) / 2.;
    //float des_leg_ang_vel = (state->leg_velocityA + state->leg_velocityB) / 2.;

    // Deadband for force control.
    /*if ( ABS( des_leg_ang - (state->motor_angleA + state->motor_angleB) / 2. ) < 0.015 )
    {
        des_leg_ang = (state->motor_angleA + state->motor_angleB) / 2.;
    }

    if ( ABS(des_leg_ang_vel ) < 0.1 )
    {
        des_leg_ang_vel = 0.;
    }*/

    //float des_mtr_angA = des_leg_ang - PI + acos(des_leg_len);
    //float des_mtr_angB = des_leg_ang + PI - acos(des_leg_len);

    // Compute the leg torque for zero hip moment and maintaining hopping height.
    //output->motor_torqueA = HUBICKI_CONTROLLER_DATA(data)->stance_p_gain * (des_mtr_angA - state->motor_angleA)
    //  + HUBICKI_CONTROLLER_DATA(data)->stance_d_gain * (des_leg_ang_vel - state->motor_velocityA) + HUBICKI_ESTIMATED_SPRING_STIFFNESS * spring_defA / HUBICKI_ESTIMATED_GEAR_RATIO;
    //output->motor_torqueB = HUBICKI_CONTROLLER_DATA(data)->stance_p_gain * (des_mtr_angB - state->motor_angleB)
    //  + HUBICKI_CONTROLLER_DATA(data)->stance_d_gain * (des_leg_ang_vel - state->motor_velocityB) + HUBICKI_ESTIMATED_SPRING_STIFFNESS * spring_defB / HUBICKI_ESTIMATED_GEAR_RATIO;

    // Clamp the torques for now, for added safety.
    //output->motor_torqueA = CLAMP( output->motor_torqueA, -3., 3. );
    //output->motor_torqueB = CLAMP( output->motor_torqueB, -3., 3. );

        //PRINT_MSG("!!<%f> <%f>", ABS(spring_defA), ABS(spring_defB));

    //if ( ( state->zPosition - leg_length * sin( leg_angle ) > -0.02 ) && ( ( ABS(spring_defA) < HUBICKI_CONTROLLER_DATA(data)->flight_spring_threshold )
    //  && ( ABS(spring_defB) < HUBICKI_CONTROLLER_DATA(data)->flight_spring_threshold ) ) )
    if ( state->toe_switch == 1 )
    {
        time_of_last_stance = eclapsedTime;
    }

    //if ( state->toe_switch == 0 && 1000 < (HUBICKI_CONTROLLER_STATE(state)->time - HUBICKI_CONTROLLER_STATE(state)->time_of_last_stance))
    if ( state->toe_switch == 0 )
    {
        // Check to see if lift off has occured.

        //PRINT_MSG("LO!\n");

        in_flight = true;

        // Reset peak height.
        peak_ht = 0.;

        after_mid_stance = false;
    }
}

void controllerTakedown() {

}
