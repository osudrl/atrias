//Test Controller

#include <atrias_controllers/controller.h>

#define ANGLE_CALCULATION_A(A) (((-90 + (A))/180.0)*PI)
#define ANGLE_CALCULATION_B(A) (((270 - (A))/180.0)*PI)

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    TEST_CONTROLLER_STATE(state)->motors_powered = false;
    output->motor_torqueA = output->motor_torqueB = 0.0;
    PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    if (TEST_CONTROLLER_STATE(state)->motors_powered)
    {
        if(input->zPosition >= TEST_CONTROLLER_DATA(data)->heightOff)
        {
            TEST_CONTROLLER_STATE(state)->jumped = true;
        }        
        if (input->zPosition <= TEST_CONTROLLER_DATA(data)->heightOff && input->zVelocity < 0 && TEST_CONTROLLER_STATE(state)->jumped)
        {
            PRINT_MSG("MOTORS OFF\n");
            TEST_CONTROLLER_STATE(state)->motors_powered = false;
            TEST_CONTROLLER_STATE(state)->jumped = false;
        }
        output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->gainP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->longLegAngle) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->gainD * input->motor_velocityA);
        output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->gainP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->longLegAngle) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->gainD * input->motor_velocityB);
    }
    else
    {
        output->motor_torqueA = output->motor_torqueB = 0.0;
        if (input->zPosition <= TEST_CONTROLLER_DATA(data)->heightOn)
        {
            PRINT_MSG("MOTORS ON: %f, %f.\n", (TEST_CONTROLLER_DATA(data)->gainP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->longLegAngle) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->gainD * input->motor_velocityA), (TEST_CONTROLLER_DATA(data)->gainP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->longLegAngle) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->gainD * input->motor_velocityB));
            TEST_CONTROLLER_STATE(state)->motors_powered = true;
        }
    }
    
}

extern void takedown_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    PRINT_MSG("Test Controller Stopped.\n");
}