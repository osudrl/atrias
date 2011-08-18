//Test Controller

#include <atrias_controllers/controller.h>

#define ANGLE_CALCULATION_A(A) (((-90 + (A))/180.0)*PI)
#define ANGLE_CALCULATION_B(B) (((270 - (B))/180.0)*PI)

#define INVERT_A(A) ((((A)/PI)*180)+90)
#define INVERT_B(B) ((((-1*(B))/PI)*180)-270)

void set_torques(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data, bool on);

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    TEST_CONTROLLER_STATE(state)->motors_powered = true;
    TEST_CONTROLLER_STATE(state)->jumped = false;
    PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    if ((input->motor_angleA <= ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->longLegAngle)) && (input->motor_angleB >= ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->longLegAngle)) && TEST_CONTROLLER_STATE(state)->motors_powered == true)
    {
        TEST_CONTROLLER_STATE(state)->motors_powered = false;
 //       PRINT_MSG("Motors OFF::");
   //     PRINT_MSG("[%f] [%f]::", input->motor_angleA, input->motor_angleB);
   // PRINT_MSG("[%f] [%f]::", ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->shortLegAngle), ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->shortLegAngle));
   // PRINT_MSG("[%f] [%f]\n", ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->longLegAngle), ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->longLegAngle));
    }
    /*if ((input->motor_angleA >= TEST_CONTROLLER_DATA(data)->longLegAngle) && (input->motor_angleB >= TEST_CONTROLLER_DATA(data)->longLegAngle) && (input->zVelocity <= 0) && (TEST_CONTROLLER_STATE(state)->jumped))
    {
        TEST_CONTROLLER_STATE(state)->jumped = false;
        TEST_CONTROLLER_STATE(state)->motors_powered = false;
        PRINT_MSG("Jumped: FALSE, Motors: FALSE\n");
    }*/
    if ((input->motor_angleA >= ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->shortLegAngle)) && (input->motor_angleB <= ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->shortLegAngle)) && TEST_CONTROLLER_STATE(state)->motors_powered == false)
    {
        TEST_CONTROLLER_STATE(state)->motors_powered = true;
        PRINT_MSG("Motors ON::");
        PRINT_MSG("[%f] [%f]::", input->motor_angleA, input->motor_angleB);
    PRINT_MSG("[%f] [%f]::", ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->shortLegAngle), ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->shortLegAngle));
    PRINT_MSG("[%f] [%f]\n", ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->longLegAngle), ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->longLegAngle));
    }
    
    //PRINT_MSG("[%f] [%f]::", input->motor_angleA, input->motor_angleB);
    //PRINT_MSG("[%f] [%f]::", ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->shortLegAngle), ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->shortLegAngle));
    //PRINT_MSG("[%f] [%f]\n", ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->longLegAngle), ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->longLegAngle));
    
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