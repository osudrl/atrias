// Test Controller
// By: Colan Dray and Michael Anderson

#include <atrias_controllers/controller.h>

#define ANGLE_CALCULATION_A(AAA) (PI/2.0 - PI + acos((AAA)))
#define ANGLE_CALCULATION_B(BBB) (PI/2.0 + PI - acos((BBB)))

#ifdef COMPILE_FOR_RTAI
    #define ROS_DEBUG(...) NULL
    #define ROS_INFO(...) NULL
    #define ROS_WARN(...) NULL
    #define ROS_ERROR(...) NULL
    #define ROS_FATAL(...) NULL
#else
    #include <ros/console.h>
#endif

int controllerIteration = 1;
int debugTimer = 100;

enum {AIR_PHASE, SHORT_LEG_STANCE_PHASE, LONG_LEG_STANCE_PHASE};

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageANew = 0.0;
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageBNew = 0.0;
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageAOld = 0.0;
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageBOld = 0.0;
    TEST_CONTROLLER_STATE(state)->currentState = 0;
    
    int i;
    for(i = 0; i <= 19; i++)
    {
        TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[i] = 0.0;
        TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[i] = 0.0;
    }
    ROS_INFO("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    TEST_CONTROLLER_STATE(state)->legAngle = (input->leg_angleA + input->leg_angleB) / 2.;
    TEST_CONTROLLER_STATE(state)->legLength = -0.5 * sin(input->leg_angleA) - 0.5 * sin(input->leg_angleB);

    if (controllerIteration > 0) {
        TEST_CONTROLLER_STATE(state)->springDeflectionAverageAOld = TEST_CONTROLLER_STATE(state)->springDeflectionAverageANew;
        TEST_CONTROLLER_STATE(state)->springDeflectionAverageBOld = TEST_CONTROLLER_STATE(state)->springDeflectionAverageBNew;
    }
    
    int i;
    
    for(i = controllerIteration - 1; i > 0; i--)
    {
        TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[i] = TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[i-1];
        TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[i] = TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[i-1];
    }
    
    TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[0] =  (input->motor_angleA - input->leg_angleA);
    TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[0] = -(input->motor_angleB - input->leg_angleB);
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageANew += (TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[0] - ((controllerIteration > 1) ? TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsA[controllerIteration-1] : 0)) / (float)controllerIteration;
    TEST_CONTROLLER_STATE(state)->springDeflectionAverageBNew += (TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[0] - ((controllerIteration > 1) ? TEST_CONTROLLER_STATE(state)->previousSpringDeflectionsB[controllerIteration-1] : 0)) / (float)controllerIteration;
    
    int curState = TEST_CONTROLLER_STATE(state)->currentState;
    
    if (debugTimer >= 100) {
        ROS_INFO("Current State: %i", curState);
        //ROS_INFO("Avg Spring Def: A: %f, B: %f", TEST_CONTROLLER_STATE(state)->springDeflectionAverageAOld, TEST_CONTROLLER_STATE(state)->springDeflectionAverageBOld);
        //ROS_INFO("New Spring Def: A: %f, B: %f", TEST_CONTROLLER_STATE(state)->springDeflectionAverageANew, TEST_CONTROLLER_STATE(state)->springDeflectionAverageBNew);
        debugTimer = 0;
    }
    else {
        debugTimer++;
    }
    
    switch(TEST_CONTROLLER_STATE(state)->currentState)
    {
        case AIR_PHASE:
            if(input->zPosition - TEST_CONTROLLER_STATE(state)->legLength * sin(TEST_CONTROLLER_STATE(state)->legAngle) < TEST_CONTROLLER_DATA(data)->toeSwitchThreshold)
            {
                TEST_CONTROLLER_STATE(state)->currentState = SHORT_LEG_STANCE_PHASE;
                debugTimer = 0;
                ROS_INFO("Begin short leg stance phase\n");
            }
            output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->flightKP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleA)) - ((TEST_CONTROLLER_DATA(data)->flightKD * input->motor_velocityA));
            output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->flightKP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleB)) - ((TEST_CONTROLLER_DATA(data)->flightKD * input->motor_velocityB));
            //ROS_INFO("Torque A: %f    B: %f    Current Def: %f", (TEST_CONTROLLER_DATA(data)->flightKP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->flightKD * input->motor_velocityA), (TEST_CONTROLLER_DATA(data)->flightKP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->flightKD * input->motor_velocityB), ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthShort));
            break;
        case SHORT_LEG_STANCE_PHASE:
            if((TEST_CONTROLLER_STATE(state)->springDeflectionAverageANew > TEST_CONTROLLER_STATE(state)->springDeflectionAverageAOld) && (TEST_CONTROLLER_STATE(state)->springDeflectionAverageBNew > TEST_CONTROLLER_STATE(state)->springDeflectionAverageBOld))
            {
                TEST_CONTROLLER_STATE(state)->currentState = LONG_LEG_STANCE_PHASE;
                debugTimer = 0;
                ROS_INFO("Begin long leg stance phase\n");
            }
            output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityA);
            output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityB);
            //ROS_INFO("Torque A: %f    B: %f", (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityA), (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->desiredLengthShort) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityB));
            break;
        case LONG_LEG_STANCE_PHASE:
            if(input->zPosition - TEST_CONTROLLER_STATE(state)->legLength * sin(TEST_CONTROLLER_STATE(state)->legAngle) > TEST_CONTROLLER_DATA(data)->toeSwitchThreshold)
            {
                TEST_CONTROLLER_STATE(state)->currentState = AIR_PHASE;
                debugTimer = 0;
                ROS_INFO("Begin air phase\n");
            }
            output->motor_torqueA = (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_A(TEST_CONTROLLER_DATA(data)->desiredLengthLong) - input->motor_angleA)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityA);
            output->motor_torqueB = (TEST_CONTROLLER_DATA(data)->stanceKP * (ANGLE_CALCULATION_B(TEST_CONTROLLER_DATA(data)->desiredLengthLong) - input->motor_angleB)) - (TEST_CONTROLLER_DATA(data)->stanceKD * input->motor_velocityB);
            break;
        default:
            ROS_ERROR("[FATAL] This line shouldn't ever be printed.\n");
            break;
    }
    if (controllerIteration < 10)
        controllerIteration++;
}

extern void takedown_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
    output->motor_torqueA = output->motor_torqueB = 0.0;
    ROS_INFO("Test Controller Stopped.\n");
}
