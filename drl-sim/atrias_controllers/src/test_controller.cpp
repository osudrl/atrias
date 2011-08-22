// Test Controller
// By: Colan Dray

#include <atrias_controllers/controller.h>

#define ANGLE_CALCULATION_A(AAA) (PI/2.0 - PI + acos((AAA)))
#define ANGLE_CALCULATION_B(BBB) (PI/2.0 + PI - acos((BBB)))
#define MAGIC_1 10

enum {AIR_PHASE, SHORT_LEG_STANCE_PHASE, LONG_LEG_STANCE_PHASE};

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageA = 0.0;
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageB = 0.0;
int i;
    for(i = 0; i < MAGIC_1; i++)
    {
        TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[i] = 0.0;
        TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[i] = 0.0;
    }
    PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{    
    TEST_CONTROLLER_STATE(state)->legAngle = (input->leg_angleA + input->leg_angleB) / 2.;
    TEST_CONTROLLER_STATE(state)->legLength = -0.5 * sin(input->leg_angleA) - 0.5 * sin(input->leg_angleB);

    TEST_CONTROLLER_STATE(state)->springDeflectionAverageA = TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[9];
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageB = TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[9];
int i;
    for(i = 0; i < MAGIC_1 - 1; i++)
    {
        TEST_CONTROLLER_STATE(state)->springDeflectionAverageA += (TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[i+1] = TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[i]);
        TEST_CONTROLLER_STATE(state)->springDeflectionAverageB += (TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[i+1] = TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[i]);
    }
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageA += (TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[0] = input->leg_angleA - input->motor_angleA);
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageB += (TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[0] = input->leg_angleB - input->motor_angleB);
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageA /= MAGIC_1 + 1;
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageB /= MAGIC_1 + 1;
    
    switch(TEST_CONTROLLER_STATE(state)->currentState)
    {
        case AIR_PHASE:
            if(input->zPosition - TEST_CONTROLLER_STATE(state)->legLength * sin(TEST_CONTROLLER_STATE(state)->legAngle) < TEST_CONTROLLER_DATA(data)->toeSwitchThreshold)
            {
                TEST_CONTROLLER_STATE(state)->currentState = SHORT_LEG_STANCE_PHASE;
                //PRINT_MSG("Begin SLS phase\n");
            }
            output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->flightKP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->flightKD * input->motor_velocityA);
            output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->flightKP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->flightKD * input->motor_velocityB);
            break;
        case SHORT_LEG_STANCE_PHASE:
            if(TEST_CONTROLLER_STATE(state)->springDeflectionAverageA < TEST_CONTROLLER_DATA(data)->springDeflectionThreshold && TEST_CONTROLLER_STATE(state)->springDeflectionAverageB < TEST_CONTROLLER_DATA(data)->springDeflectionThreshold)
            {
                TEST_CONTROLLER_STATE(state)->currentState = LONG_LEG_STANCE_PHASE;
                //PRINT_MSG("Begin LLS phase\n");
            }
            output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityA);
            output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityB);
            break;
        case LONG_LEG_STANCE_PHASE:
            if(input->zPosition - TEST_CONTROLLER_STATE(state)->legLength * sin(TEST_CONTROLLER_STATE(state)->legAngle) > TEST_CONTROLLER_DATA(data)->toeSwitchThreshold)
            {
                TEST_CONTROLLER_STATE(state)->currentState = AIR_PHASE;
                //PRINT_MSG("Begin AIR phase\n");
            }
            output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthLong) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityA);
            output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->desiredLengthLong) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityB);
            break;
        default:
            PRINT_MSG("[FATAL] This line shouldn't ever be printed.\n");
            break;
    }
}

extern void takedown_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    PRINT_MSG("Test Controller Stopped.\n");
}
