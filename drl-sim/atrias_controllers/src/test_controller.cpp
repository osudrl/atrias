//Test Controller

#include <atrias_controllers/controller.h>

#define ANGLE_CALCULATION_A(A) (((-90 + (A))/180.0)*PI)
#define ANGLE_CALCULATION_B(A) (((270 - (A))/180.0)*PI)

void set_torques(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data, bool on);

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    TEST_CONTROLLER_STATE(state)->motors_powered = true;
    TEST_CONTROLLER_STATE(state)->jumped = true;
    PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    if (input->zPosition >= TEST_CONTROLLER_DATA(data)->heightOff && input->zVelocity >= 0)
    {
        TEST_CONTROLLER_STATE(state)->jumped = true;
    }
    if (input->zPosition <= TEST_CONTROLLER_DATA(data)->heightOff && input->zVelocity <= 0 && TEST_CONTROLLER_STATE(state)->jumped)
    {
        TEST_CONTROLLER_STATE(state)->jumped = false;
        TEST_CONTROLLER_STATE(state)->motors_powered = false;
    }
    if (input->zPosition <= TEST_CONTROLLER_DATA(data)->heightOn)
    {
        TEST_CONTROLLER_STATE(state)->motors_powered = true;
    }
    
    set_torques(input, output, state, data, TEST_CONTROLLER_STATE(state)->motors_powered);
}
//PRINT_MSG("MOTORS ON: %f, %f.\n", (TEST_CONTROLLER_DATA(data)->gainP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->longLegAngle) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->gainD * input->motor_velocityA), (TEST_CONTROLLER_DATA(data)->gainP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->longLegAngle) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->gainD * input->motor_velocityB));

extern void takedown_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    PRINT_MSG("Test Controller Stopped.\n");
}

void set_torques(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data, bool on)
{
    if(on)
    {
        output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->gainP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->longLegAngle) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->gainD * input->motor_velocityA);
        output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->gainP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->longLegAngle) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->gainD * input->motor_velocityB);
    }
    else
    {
        output->motor_torqueA = output->motor_torqueB = 0.0;
    }
}