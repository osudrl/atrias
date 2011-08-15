//Test Controller

#include <atrias_controllers/controller.h>

void flight_state_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data);
void stance_state_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data);

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    TEST_CONTROLLER_STATE(state)->in_flight = true;
    output->motor_torqueA = output->motor_torqueB = 0.0;
    PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    if (TEST_CONTROLLER_STATE(state)->in_flight)
    {
        flight_state_controller(input, output, state, data);
        if (input->toe_switch)
        {
            PRINT_MSG("Test controller status: LANDED.\n");
            TEST_CONTROLLER_STATE(state)->in_flight = false;
        }
    }
    else
    {
        stance_state_controller(input, output, state, data);
        if (!input->toe_switch)
        {
            PRINT_MSG("Test controller status: TAKEOFF.\n");
            TEST_CONTROLLER_STATE(state)->in_flight = true;
        }
    }
}

extern void takedown_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    PRINT_MSG("Test Controller Stopped.\n");
}

void flight_state_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->flight_motor_gain_p * (TEST_CONTROLLER_DATA(data)->flight_desired_motor_angleA - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->flight_motor_gain_d * input->motor_velocityA);
    output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->flight_motor_gain_p * (TEST_CONTROLLER_DATA(data)->flight_desired_motor_angleB - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->flight_motor_gain_d * input->motor_velocityB);
}

void stance_state_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->stance_motor_gain_p * (TEST_CONTROLLER_DATA(data)->stance_desired_motor_angleA - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->stance_motor_gain_d * input->motor_velocityA);
    output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->stance_motor_gain_p * (TEST_CONTROLLER_DATA(data)->stance_desired_motor_angleB - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->stance_motor_gain_d * input->motor_velocityB);
}