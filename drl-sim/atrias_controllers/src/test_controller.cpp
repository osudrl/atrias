//Test Controller

#include <atrias_controllers/controller.h>

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    float leg_angle = (input->leg_angleA + input->leg_angleB) / 2.;
    float leg_length = - 0.5 * sin(input->leg_angleA) - 0.5 * sin(input->leg_angleB);
    if(input->zPosition - leg_length * sin(leg_angle) < 0.02)
    {
        if((input->leg_angleA - input->motor_angleA) >= TEST_CONTROLLER_DATA(data)->activationDeflection || (input->leg_angleB - input->motor_angleB) >= TEST_CONTROLLER_DATA(data)->activationDeflection)
        {
            output->motor_torqueA = TEST_CONTROLLER_DATA(data)->stanceGainP * ((-1)*(PI/2) + acos(TEST_CONTROLLER_DATA(data)->desiredLength));
            output->motor_torqueB = TEST_CONTROLLER_DATA(data)->stanceGainP * (( 3)*(PI/2) + acos(TEST_CONTROLLER_DATA(data)->desiredLength));
            //output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->stanceGainP * (((-1)*(PI/2) + acos(TEST_CONTROLLER_DATA(data)->desiredLength)) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->stanceGainD * input->motor_velocityA);
            //output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->stanceGainP * ((( 3)*(PI/2) - acos(TEST_CONTROLLER_DATA(data)->desiredLength)) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->stanceGainD * input->motor_velocityB);
        }
        else
        {
        output->motor_torqueA = output->motor_torqueB = 0.0;
        }
    }
    else
    {
        output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->flightGainP * (((-1)*(PI/2) + acos(TEST_CONTROLLER_DATA(data)->desiredLength)) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->flightGainD * input->motor_velocityA);
        output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->flightGainP * ((( 3)*(PI/2) - acos(TEST_CONTROLLER_DATA(data)->desiredLength)) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->flightGainD * input->motor_velocityB);
    }
    
    
    //(ABS(spring_deflection_A) > RAIBERT_CONTROLLER_DATA(data)->stance_spring_threshold)
    //(ABS(spring_deflection_B) > RAIBERT_CONTROLLER_DATA(data)->stance_spring_threshold)
    
    //if (  && (  ||  ) )
}

extern void takedown_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    PRINT_MSG("Test Controller Stopped.\n");
}