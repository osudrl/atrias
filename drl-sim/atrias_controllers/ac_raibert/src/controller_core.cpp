/*
 * controller_core.cpp
 *
 * FORCE CONTROL FOR VERTICAL HOPPING - ATRIAS 2.0 - By Mikhail Jones
 * Code is originally from Devin Koepl's Raibert controller for ATRIAS 1.0
 *
 * Ported to new controller system by Michael Anderson
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <ac_raibert/controller_core.h>

ControllerInitResult controllerInit() {
    in_flight = true;
    stance_time = 0.0;
    spring_def_sumA = 0.0;
    spring_def_sumB = 0.0;

	ControllerInitResult cir;
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = sizeof(ControllerStatus);
    cir.error = false;
    return cir;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);
    ControllerStatus st;

    //float stance_trigger_height = 0.913;   // For floor height
    //float stance_trigger_height = 0.904;   // For foam 1 height
    float stance_trigger_height = 0.910;  // For foam 2 height (trial 1)
    //float stance_trigger_height = 0.913;  // For foam 2 height (trial 2)
    //float stance_trigger_height = 0.915;  // For foam 2 height (trial 3)
    //float stance_trigger_height = 0.85;  // For rocks height

    // UPDATE LEG CONTROL
    if (in_flight)
    {
        // We reset the integral error sums for each hop.
        spring_def_sumA = 0.0;
        spring_def_sumB = 0.0;

        // Call flight controller.
        flight_controller(&state, id, output);

        // Check to see if we are still in flight, if not we go into stance.
        // Based on spring deflection. (Will not work as is for multiple hops with current flight spring oscillations)
        //if ((ABS(input->motor_angleA - input->leg_angleA) > 0.007) && (ABS(input->motor_angleB - input->leg_angleB) > 0.007))
        // Based on toe switch.
        //if ((input->toe_switch == 1.0) && (RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce >= 3.0))
        // Based on z position. (Will not work as is for horizontal hopping)
        if (state.zPosition <= stance_trigger_height)
        {
            in_flight = false;
        }
    }
    else
    {
        // Call stance controller.
        stance_controller(&state, id, output);

        // Check to see if we are in stance, if not we go into flight.
        // Based on toe switch and time of stance.
        //if  ((input->toe_switch == 0.0) && (RAIBERT_CONTROLLER_STATE(state)->stance_time > 50.0))
        // Based on toe switch.
        //if ((input->toe_switch == 0.0) && (RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce >= 3.0))
        // Based on z position and time of stance.
        if  ((state.zPosition > stance_trigger_height) && (stance_time > 50.0))
        {
            in_flight = true;
        }
    }

    // UPDATE HIP CONTROL
    // We control the hip the same in flight and stance, the equation is based on experimental test to prevent lateral forces on the knee joints.
    float desired_hip_angle = 0.99366 * state.body_angle + 0.03705;
    desired_hip_angle = CLAMP(desired_hip_angle, -0.2007, 0.148);

    // Hip PD control.
    output->motor_torque_hip = (id->stance_hip_p_gain * (desired_hip_angle - state.motor_angle_hip)) - (id->stance_hip_d_gain * state.motor_velocity_hip);

    // MISC
    // Make data avaiable to log.
    state.phase = in_flight;

    st.in_flight = in_flight;
    structToByteArray(st, status);
}

// FLIGHT CONTROLLER
void flight_controller(robot_state* state, InputData* id, ControllerOutput* output)
{
    // Reset stance based variables.
    stance_time = 0.0;
    spring_def_sumA = 0.0;
    spring_def_sumB = 0.0;

    // Get values from GUI
    float leg_angle_gain = id->hor_vel_gain;

    // Desired leg angle during flight.
    float desired_leg_angle = PI / 2.0 + leg_angle_gain;

    // Calculate desired motor angles during flight.
    state->desired_motor_angleA = desired_leg_angle - PI + acos(id->preferred_leg_len);
    state->desired_motor_angleB = desired_leg_angle + PI - acos(id->preferred_leg_len);

    // Leg PD control.
    output->motor_torqueA = (id->flight_p_gain * (state->desired_motor_angleA - state->motor_angleA)) - (id->flight_d_gain * state->motor_velocityA);
    output->motor_torqueB = (id->flight_p_gain * (state->desired_motor_angleB - state->motor_angleB)) - (id->flight_d_gain * state->motor_velocityB);

    state->desired_spring_defA = 0.0;
    state->desired_spring_defB = 0.0;
}



// STANCE CONTROLLER
void stance_controller(robot_state* state, InputData* id, ControllerOutput* output)
{
    //*******************************
    // CONTROL SWITCHER!!!!
    //*******************************

    // 0 -> FORCE CONTROL,  1 -> POSITION CONTROL
    int STANCE_POSITION_CONTROL = 0;

    //*********************
    //   SAFETY VALUES
    //*********************

    // Values for safety limits.  If outside of these, a damping controller engages

    // Leg length limits
    float safety_length_long = 0.97;
    float safety_length_short = 0.6;

    //  Motor A angle limits
    float safety_angleA_short = -2./3.*PI;
    float safety_angleA_long = 0.;

    // Motor B angle limits
    float safety_angleB_short = PI;
    float safety_angleB_long = 5./3.*PI;

    //*********************
    //     CONSTANTS
    //*********************

    float SPRING_CONSTANT = 1301.9;

    //************************
    //  STATE CALCULATIONS
    //***********************

    // Leg length calculation
    float leg_length = cos((2.0 * PI + state->leg_angleA - state->leg_angleB ) / 2.0);

    // Calculate spring deflections.
    float spring_defA = state->motor_angleA - state->leg_angleA;
    float spring_defB = state->motor_angleB - state->leg_angleB;


    //************************
    //      GUI VALUES
    //************************

    // Get values from GUI
    float HOP_GAIN = id->des_hop_ht;
    float FORCE_P_GAIN = id->hop_ht_gain;
    float FORCE_D_GAIN = id->leg_ang_gain;
    float FORCE_I_GAIN = id->des_hor_vel;


    //****************************************
    //  FORCE/TORQUE/DEFLECTION COMPUTATIONS
    //****************************************

    // Quartic Coefficients fitted to MATLAB Simulation Force Profile

    // COEFFICIENTS FOR 1.0 meter drop height
    //float coeffA = -1.4442*pow(10,6);
    //float coeffB =  6.4643*pow(10,5);
    //float coeffC = -1.6339*pow(10,5);
    //float coeffD =  2.0380*pow(10,4);
    //float coeffE =  0.;

    // COEFFICIENTS FOR 0.95 meter drop height
    //float coeffA = -3.4031*pow(10,5);
    //float coeffB =  1.5565*pow(10,5);
    //float coeffC = -8.0154*pow(10,4);
    //float coeffD =  1.4323*pow(10,4);
    //float coeffE = -1.9488;

    // COEFFICIENTS FOR 0.96 meter drop height... computed by fitting polynomial to steady-state hopping
    float coeffA = -5.5078*pow(10,3);
    float coeffB = -1.5446*pow(10,4);
    float coeffC = -4.5040*pow(10,4);
    float coeffD =  1.1778*pow(10,4);
    float coeffE = -3.6484;

    float t = ((float)stance_time) / 1000.;
    // Quartic approximation of force profile
    float desired_force = CLAMP(coeffA*pow(t,4) + coeffB*pow(t,3) + coeffC*pow(t,2) + coeffD*pow(t,1) + coeffE, 0, 1500);

    // DEBUG PRINT FORCE
    //printk("Desired Force: %d \n", (int) desired_force);
    // END DEBUG

    // Forces to Torques based on current kinematic configuration
    float desired_torque = 0.5*desired_force*cos(PI + asin(leg_length));

    // Desired Spring Deflections
    state->desired_spring_defA = CLAMP(HOP_GAIN * (desired_torque/SPRING_CONSTANT), -0.3, 0.0);
    state->desired_spring_defB = CLAMP(HOP_GAIN * (-desired_torque/SPRING_CONSTANT), 0.0, 0.3);

    // Spring deflection profile based on data from a 1m z-position drop with 0.90m leg length.
    //input->desired_spring_defA = CLAMP(HOP_GAIN * (0.0000149620790 * pow(RAIBERT_CONTROLLER_STATE(state)->stance_time, 2.0) - 0.00318251866 * RAIBERT_CONTROLLER_STATE(state)->stance_time), -0.3, 0.0);
    //input->desired_spring_defB = CLAMP(HOP_GAIN * (-0.0000149620790 * pow(RAIBERT_CONTROLLER_STATE(state)->stance_time, 2.0) + 0.00318251866 * RAIBERT_CONTROLLER_STATE(state)->stance_time), 0.0, 0.3);

    //************************
    //    CONTROL SIGNAL
    //************************

    // Compute feed-forward torque signal for force tracking (Soley based on spring deflection, later could include motor velocity and inertia.)
    float feed_forward_torqueA = 310.0 * state->desired_spring_defA;
    float feed_forward_torqueB = 310.0 * state->desired_spring_defB;

    // Deflection sum error for integral gain
    spring_def_sumA = CLAMP(spring_def_sumA + (state->desired_spring_defA - spring_defA), -10.0, 10.0);
    spring_def_sumB = CLAMP(spring_def_sumB + (state->desired_spring_defB - spring_defB), -10.0, 10.0);

    //*****************************************************************
    // POSITION CONTROL COMPUTATIONS
    //*****************************************************************

    // Get values from GUI
    float leg_angle_gain = id->hor_vel_gain;
    // SAME DESIRED LEG ANGLE AS FLIGHT CONTROLLER
    float desired_leg_angle = PI / 2.0 + leg_angle_gain;

    // POSITION COEFFICIENTS FOR 0.96 meter drop height... computed by fitting polynomial to steady-state hopping
    float PcoeffA = 6.2939;
    float PcoeffB = -3.6861;
    float PcoeffC = 0.97695;
    float PcoeffD =  -0.092146;
    float PcoeffE = 0.9005;

    float desired_length = CLAMP(PcoeffA*pow(t,4) + PcoeffB*pow(t,3) + PcoeffC*pow(t,2) + PcoeffD*pow(t,1) + PcoeffE, 0.85, 0.9045);

    state->desired_motor_angleA = desired_leg_angle - PI + acos(desired_length);
    state->desired_motor_angleB = desired_leg_angle + PI - acos(desired_length);

    // FEEDFORWARD HOLDING TORQUE
    float feed_position_torque = 10.;

    //**********************************************************
    // CONTROLLING "IF" STATEMENT:  SAFETY FIRST, then selects between position and force control
    //**********************************************************

    // When leg length reaches virtual safety limits, add damping to slow motors before real hardstops are hit.
    if(leg_length != CLAMP(leg_length, safety_length_short, safety_length_long) ||
        state->motor_angleA != CLAMP(state->motor_angleA, safety_angleA_short, safety_angleA_long) ||
        state->motor_angleB != CLAMP(state->motor_angleB, safety_angleB_short, safety_angleB_long))
    {
        output->motor_torqueA = - 100.0 * state->motor_velocityA;
        output->motor_torqueB = - 100.0 * state->motor_velocityB;
    }
    else
    {
        if(STANCE_POSITION_CONTROL == 1) // POSITION CONTROL
        {
            // Leg PD control.
            output->motor_torqueA = (FORCE_P_GAIN * (state->desired_motor_angleA - state->motor_angleA)) - (FORCE_D_GAIN * state->motor_velocityA) - feed_position_torque;
            output->motor_torqueB = (FORCE_P_GAIN * (state->desired_motor_angleB - state->motor_angleB)) - (FORCE_D_GAIN * state->motor_velocityB) + feed_position_torque;
        }
        else // FORCE CONTROL
        {
            // Leg A, PID control with feed forward.
            state->desired_motor_angleA = state->motor_angleA + (state->desired_spring_defA - spring_defA);
            output->motor_torqueA = (FORCE_P_GAIN * (state->desired_motor_angleA - state->motor_angleA)) - (FORCE_D_GAIN * (state->motor_velocityA)) + (FORCE_I_GAIN * spring_def_sumA) + feed_forward_torqueA;

            // Leg B, PID control with feed forward.
            state->desired_motor_angleB = state->motor_angleB + (state->desired_spring_defB - spring_defB);
            output->motor_torqueB = (FORCE_P_GAIN * (state->desired_motor_angleB - state->motor_angleB)) - (FORCE_D_GAIN * (state->motor_velocityB)) + (FORCE_I_GAIN * spring_def_sumB) + feed_forward_torqueB;
        }
    }

    // Update stance time.
    stance_time += 1.0;
}

void controllerTakedown() {

}
