// Devin Koepl


#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <drl_library/drl_math.h>

#include <stdio.h>

#include <stdint.h>
#include <math.h>

#define TEST_CONTROLLER_MAGIC 24

#define SIZE_OF_CONTROLLER_DATA         200
#define SIZE_OF_CONTROLLER_STATE_DATA   256

//======================================================//

// Debugging print statements.

//#define DEBUG_CONTROLLERS
#undef  DEBUG_CONTROLLERS


#ifdef DEBUG_CONTROLLERS
#include <ros/ros.h>
#define PRINT_MSG   ROS_INFO
#define PRINT_WARN  ROS_WARN
#else
#define PRINT_MSG   // printf
#define PRINT_WARN  // printf
#endif

// Temporary for testing in uspace.
//#include <rtai_lxrt.h>

//#define PRINT_MSG rtai_print_to_screen
//#define PRINT_WARN rtai_print_to_screen

//======================================================//

// Controller types available.

#define NO_CONTROLLER                   0
#define MOTOR_TORQUE_CONTROLLER         1
#define MOTOR_POSITION_CONTROLLER       2
#define LEG_TORQUE_CONTROLLER           3
#define LEG_POSITION_CONTROLLER         4
#define SINE_WAVE_CONTROLLER            5
#define RAIBERT_CONTROLLER              6
#define HUBICKI_CONTROLLER              7
#define TEST_CONTROLLER                 8
#define EQU_GAIT_CONTROLLER             9
#define FORCE_CONTROLLER                10

//======================================================//

// If the controller commands both motors to have a torque below this value, assume that no controller is present,
// and command a small torque to keep the robot off of its hardstops.
#define MIN_TRQ_THRESH                  1E-9


// ============================================================================
// From old uspace_kern_shm.h
// ============================================================================

#define SHM_TO_USPACE_ENTRIES   1000000
#define SHM_TO_USPACE_MSGS      100

#define NO_MSG          0
#define INFO_MSG        1
#define WARN_MSG        2
#define ERROR_MSG       3

//======================================================//

// General control structs.

typedef struct {
    float body_angle;
    float body_angle_vel;
    float motor_angleA;
    float motor_angleA_inc;
    float motor_angleB;
    float motor_angleB_inc;
    float leg_angleA;
    float leg_angleB;

    float motor_velocityA;
    float motor_velocityB;
    float leg_velocityA;
    float leg_velocityB;

    float hip_angle;
    float hip_angle_vel;

    float xPosition;
    float yPosition;
    float zPosition;

    float xVelocity;
    float yVelocity;
    float zVelocity;

    unsigned char motor_currentA;
    unsigned char motor_currentB;

    unsigned char toe_switch;

    unsigned char command;

    float thermistorA[3];
    float thermistorB[3];
    float motorVoltageA;
    float motorVoltageB;
    float logicVoltageA;
    float logicVoltageB;
    unsigned char medullaStatusA;
    unsigned char medullaStatusB;

    float time_of_last_stance;

    float phase;
    float desired_motor_position_A;
    float desired_motor_position_B;
    float desired_def_A;
    float desired_def_B;
} ControllerInput;


typedef struct {
    float motor_torqueA;
    float motor_torqueB;
    float motor_torque_hip;
} ControllerOutput;

// This struct is where the controller can keep personal information.

typedef struct {
    unsigned char state;
    unsigned char controller_loaded;

    // Controller specific space.
    unsigned char data[SIZE_OF_CONTROLLER_STATE_DATA];
} ControllerState;

// This struct is the input to the controller.

typedef struct {
    unsigned char command;
    unsigned char controller_requested;

    // Controller specific space.
    unsigned char data[SIZE_OF_CONTROLLER_DATA];
} ControllerData;


// ============================================================================
// Data to controller wrapper.
// ============================================================================

typedef struct {
    unsigned char       control_index;
    unsigned char       req_switch;

    ControllerData      controller_data[2];

    ControllerInput     controller_input;
    ControllerOutput    controller_output;
    ControllerState     controller_state;
} ControllerWrapperData;


//======================================================//

// Specific control structs

typedef struct {
    float mtr_trqA;
    float mtr_trqB;
    float mtr_trq_hip;
} MtrTrqControllerData;

typedef struct {
    float mtr_angA;
    float mtr_angB;
    float p_gain;
    float d_gain;
} MtrPosControllerData;

typedef struct {
    float leg_ang_trq;
    float leg_len_trq;
} LegTrqControllerData;

typedef struct {
    float leg_ang;
    float leg_len;
    float hip_ang;
    float p_gain;
    float d_gain;
    float hip_p_gain;
    float hip_d_gain;
} LegPosControllerData;

typedef struct {
    float prev_angle;
} LegPosControllerState;

typedef struct {
    // Inputs
    float leg_ang_frq;
    float leg_ang_amp;
    float leg_len_frq;
    float leg_len_amp;
    float p_gain;
    float d_gain;

    // For Control
    float time;
} SinWaveControllerData;

typedef struct {
    float time;
} SinWaveControllerState;

typedef struct {
    // Inputs
    float des_hor_vel;
    float des_hop_ht;
    float hor_vel_gain;
    float hop_ht_gain;
    float leg_ang_gain;
    float stance_p_gain;
    float stance_d_gain;
    float stance_spring_threshold;
    float preferred_leg_len;
    float flight_p_gain;
    float flight_d_gain;
    float flight_spring_threshold;
    float stance_hip_p_gain;
    float stance_hip_d_gain;
    float flight_hip_p_gain;
    float flight_hip_d_gain;
} RaibertControllerData;

typedef struct {
    unsigned char in_flight;
    unsigned char after_mid_stance;
    unsigned char after_mid_flight;

    float peak_ht;
    float last_leg_len;
    float last_stance_hip_angle;

    float time;
    float time_lo;
    float time_td;

    float time_of_last_stance;
    float last_hip_angle;

    float force_sum_A;
    float force_sum_B;

    float phase;
    float desired_motor_position_A;
    float desired_motor_position_B;
    float desired_def_A;
    float desired_def_B;
} RaibertControllerState;


typedef struct {
    // Inputs
    float des_hor_vel;
    float des_hop_ht;
    float hor_vel_gain;
    float hop_ht_gain;
    float leg_ang_gain;
    float stance_p_gain;
    float stance_d_gain;
    float stance_spring_threshold;
    float preferred_leg_len;
    float flight_p_gain;
    float flight_d_gain;
    float flight_spring_threshold;
    float stance_hip_p_gain;
    float stance_hip_d_gain;
    float flight_hip_p_gain;
    float flight_hip_d_gain;
} HubickiControllerData;

typedef struct {
    unsigned char in_flight;
    unsigned char after_mid_stance;
    unsigned char after_mid_flight;

    float peak_ht;
    float last_leg_len;
    float last_stance_hip_angle;

    float time;
    float time_lo;
    float time_td;

    float time_of_last_stance;
    float last_hip_angle;
} HubickiControllerState;

typedef struct {
    float flightKP;
    float flightKD;
    float stanceKP;
    float stanceKD;
    float desiredLengthLong;
    float desiredLengthShort;
    float toeSwitchThreshold;
    float springDeflectionThreshold;
} TestControllerData;

typedef struct {
    float legLength;
    float legAngle;
    float springDeflectionAverageANew;
    float springDeflectionAverageBNew;
    float springDeflectionAverageAOld;
    float springDeflectionAverageBOld;
    float previousSpringDeflectionsA[TEST_CONTROLLER_MAGIC];
    float previousSpringDeflectionsB[TEST_CONTROLLER_MAGIC];
    float lastZPosition;
    int currentState;
} TestControllerState;

typedef struct {
    float p_gainA;
    float d_gainA;
    float i_gainA;
    float p_gainB;
    float d_gainB;
    float i_gainB;
    float spring_deflection;
} ForceControllerData;

typedef struct {
    float springDeflectionA[250];
    float springDeflectionB[250];
    float velABuf[100];
    float velBBuf[100];
    int velBufLoc;
    int ringBufferLocation;
} ForceControllerState;

// Macros for dereferencing pointers.
#define MTR_TRQ_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((MtrTrqControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define MTR_POS_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((MtrPosControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define LEG_TRQ_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((LegTrqControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define LEG_POS_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((LegPosControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define SIN_WAVE_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((SinWaveControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define RAIBERT_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((RaibertControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define HUBICKI_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((HubickiControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define TEST_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((TestControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define FORCE_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((ForceControllerData *)(&(CONTROLLER_DATA_PTR->data)))

#define MTR_TRQ_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((MtrTrqControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define MTR_POS_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((MtrPosControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define LEG_TRQ_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((LegTrqControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define LEG_POS_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((LegPosControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define SIN_WAVE_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((SinWaveControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define RAIBERT_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((RaibertControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define HUBICKI_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((HubickiControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define TEST_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((TestControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define FORCE_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((ForceControllerState *)(&(CONTROLLER_STATE_PTR->data)))

#endif // CONTROLLER_H

