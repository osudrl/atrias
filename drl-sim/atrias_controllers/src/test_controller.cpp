// Ported by: Colan Dray & Alireza Ramezani
// MATLAB code by: Jessy Grizzle

/* Stuff to remember
#include <iostream>
#include <stdlib.h>
#include <math.h>
double FCP_SETPTTDA = 0.1230;
const double FCP_SETPTTDA_NOM = 0.0524;
double ellzcmT = ellzcmTa -2.*ellzBatteryBack *mBatteryPack/(mT);
double FCP_SETPTLS = 0.7854;
*/

#include <atrias_controllers/controller.h>

//float FCP_SETPTTDA = 0.0;

#define PI 3.14159265
#define g 9.81
#define MAX_TORQUE 15.0
#define FCP_KPLS 400.0
#define FCP_KDLS 50.4077
#define FCP_KPTDA 750.0
#define FCP_KDTDA 50.0
#define FCP_SETPTTDA_NOM 0.0
#define SCP_KPLS 600.0
#define SCP_KDLS 167.8620
#define SCP_SETPTLS 0.2618
#define SCP_KPTDA 150.0
#define SCP_KDTDA 26.6597
#define SCP_SETPTTDA -0.0175
#define SCP_EDES 273.0
#define ALPHA 3.0
#define L1 0.45
#define L2 0.5
#define L3 0.5
#define L4 0.5
#define m1 0.66149
#define m2 0.68292
#define m3 0.19126
#define m4 0.42493
#define mT 44.0 / 2.0
#define R1 20.0
#define R2 20.0
#define K1 1200.0 / 0.5
#define K2 1200.0 / 0.5
#define Jcm1 0.01910
#define Jcm2 0.02116
#define Jcm3 0.00633
#define Jcm4 0.01243
#define JcmT 1.37
#define Jgear1 0.0025
#define Jgear2 0.0025
#define Jrotor1 0.00286
#define Jrotor2 0.00286
#define ellzcm1 0.16957
#define ellzcm2 0.18626
#define ellzcm3 0.24997
#define ellzcm4 0.23832
#define ellycm1 0.04566
#define ellycm2 -0.02624
#define ellycm3 0.0
#define ellycm4 0.0
#define ellzcmTa  0.01
#define ellycmT 0.0
#define mBatteryPack 6.7
#define ellzBatteryBack -0.09
#define FCP_SETPTLS 0.01 - 2.0 * -0.09 * 6.7 / 44.0 / 2.0
#define REF_SPEED 1.3
#define STANCE_CONTROLLER 2

void flight_state_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data);
void stance_state_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data);
void abs_max(float *num, float max);

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
  TEST_CONTROLLER_STATE(state)->in_flight = true;
  output->motor_torqueA = output->motor_torqueB = 0.0;
  PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
  //TEST_CONTROLLER_STATE(state)->torso = input->body_angle - PI / 2;
  //TEST_CONTROLLER_STATE(state)->motorA = input->motor_angleA - TEST_CONTROLLER_STATE(state)->torso + PI / 2;
  //TEST_CONTROLLER_STATE(state)->motorB = input->motor_angleB - TEST_CONTROLLER_STATE(state)->torso + PI / 2;
  //TEST_CONTROLLER_STATE(state)->legA = input->leg_angleA - TEST_CONTROLLER_STATE(state)->torso + PI / 2;
  //TEST_CONTROLLER_STATE(state)->legB = input->leg_angleB - TEST_CONTROLLER_STATE(state)->torso + PI / 2;
  //TEST_CONTROLLER_STATE(state)->dtorso = input->body_ang_vel;
  //TEST_CONTROLLER_STATE(state)->dmotorA = input->motor_velocityA;
  //TEST_CONTROLLER_STATE(state)->dmotorB = input->motor_velocityB;
  //TEST_CONTROLLER_STATE(state)->dlegA = input->leg_velocityA;
  //TEST_CONTROLLER_STATE(state)->dlegB = input->leg_velocityB;
  //TEST_CONTROLLER_STATE(state)->Z = input->height;
  //TEST_CONTROLLER_STATE(state)->dZ = input->vertical_velocity;
  //
  //TEST_CONTROLLER_STATE(state)->sprdefA = ABS(TEST_CONTROLLER_STATE(state)->motorA - TEST_CONTROLLER_STATE(state)->legA);
  //TEST_CONTROLLER_STATE(state)->sprdefB = ABS(TEST_CONTROLLER_STATE(state)->motorB - TEST_CONTROLLER_STATE(state)->legB);
  //TEST_CONTROLLER_STATE(state)->threshF = TEST_CONTROLLER_DATA(data)->flight_threshold;
  //TEST_CONTROLLER_STATE(state)->threshS = TEST_CONTROLLER_DATA(data)->stance_threshold;
  //TEST_CONTROLLER_STATE(state)->motgain = TEST_CONTROLLER_DATA(data)->motor_gain;

  if(TEST_CONTROLLER_STATE(state)->in_flight)
    {
      flight_state_controller(input, output, state, data);
      if (input->motor_angleA - input->leg_angleA > TEST_CONTROLLER_DATA(data)->flight_threshold || input->motor_angleB - input->leg_angleB > TEST_CONTROLLER_DATA(data)->flight_threshold)
	{
	  PRINT_MSG("Test controller status: LANDED.");
	  TEST_CONTROLLER_STATE(state)->in_flight = false;
	}
    }
  else
    {
      stance_state_controller(input, output, state, data);
      if (input->motor_angleA - input->leg_angleA < TEST_CONTROLLER_DATA(data)->stance_threshold || input->motor_angleB - input->leg_angleB < TEST_CONTROLLER_DATA(data)->stance_threshold)
	{
	  PRINT_MSG("Test controller status: TAKEOFF.");
	  TEST_CONTROLLER_STATE(state)->in_flight = true;

	  /*
	    TEST_CONTROLLER_STATE(state)->error_speed = REF_SPEED - ((mT*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*L2+cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4-sin(TEST_CONTROLLER_STATE(state)->torso)*ellycmT-cos(TEST_CONTROLLER_STATE(state)->torso)*ellzcmT)+m1*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*L2+cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4-sin(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*ellycm1-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*ellzcm1)+m2*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*L2+cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4-sin(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*ellycm2-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*ellzcm2)+m3*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*L2+cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L1-sin(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*ellycm3-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*ellzcm3)+m4*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4-sin(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*ellycm4-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*ellzcm4))/(mT+m1+m2+m3+m4)*TEST_CONTROLLER_STATE(state)->dtorso+(mT*cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4+m1*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4-sin(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*ellycm1-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*ellzcm1)+m2*cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4+m3*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L1)+m4*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*L4-sin(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*ellycm4-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorA)*ellzcm4))/(mT+m1+m2+m3+m4)*TEST_CONTROLLER_STATE(state)->dmotorA+(mT*cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*L2+m1*cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*L2+m2*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*L2-sin(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*ellycm2-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*ellzcm2)+m3*(cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*L2-sin(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*ellycm3-cos(TEST_CONTROLLER_STATE(state)->torso+TEST_CONTROLLER_STATE(state)->motorB)*ellzcm3))/(mT+m1+m2+m3+m4)*TEST_CONTROLLER_STATE(state)->dmotorB);

	    if(TEST_CONTROLLER_STATE(state)->error_speed > 0.5)
	    {
	    TEST_CONTROLLER_STATE(state)->FCP_SETPTTDA = -1.0 * PI / 180;
	    }
	    else if(TEST_CONTROLLER_STATE(state)->error_speed < -0.3)
	    {
	    TEST_CONTROLLER_STATE(state)-> FCP_SETPTTDA = FCP_SETPTTDA_NOM - 12.0 * TEST_CONTROLLER_STATE(state)->error_speed * PI / 180;
	    }
	    else
	    {
	    FTEST_CONTROLLER_STATE(state)->CP_SETPTTDA = ((TEST_CONTROLLER_STATE(state)->error_speed < 0)? -1 : 1) * FCP_SETPTTDA_NOM;
	    }
	  */
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
  /*
  // Math from file
  float vHip2 = TEST_CONTROLLER_STATE(state)->dZ;
  float p42 = TEST_CONTROLLER_STATE(state)->Z + cos(TEST_CONTROLLER_STATE(state)->torso + TEST_CONTROLLER_STATE(state)->motorB) * L2 + cos(TEST_CONTROLLER_STATE(state)->torso + TEST_CONTROLLER_STATE(state)->motorA) * L4;
  float qLA = (TEST_CONTROLLER_STATE(state)->motorA + TEST_CONTROLLER_STATE(state)->motorB) / 2;
  float dqLA = (TEST_CONTROLLER_STATE(state)->dmotorA + TEST_CONTROLLER_STATE(state)->dmotorB) / 2;
  float qLS = TEST_CONTROLLER_STATE(state)->legB - TEST_CONTROLLER_STATE(state)->legA;
  float dqLS = TEST_CONTROLLER_STATE(state)->dlegB - TEST_CONTROLLER_STATE(state)->dlegA;
  float qLA_abs = qLA + TEST_CONTROLLER_STATE(state)->torso;
  float qTDA = qLA_abs - PI;
  float dqLA_abs = dqLA + TEST_CONTROLLER_STATE(state)->dtorso;
  float dqTDA = dqLA_abs;
  float y2 = FCP_SETPTTDA - qTDA;
  float dy2 = 0 - dqTDA;
  float uLA = FCP_KPTDA * y2 + FCP_KDTDA * dy2;
  float y1, dy1, uLS;
  if( (vHip2 > 0) && (p42 < 0.05) )
  {
  FCP_SETPTLS += (10.0 * PI / 180);
  y1 = FCP_SETPTLS - qLS;
  dy1 = 0 - dqLS;
  uLS = FCP_KPLS * y1 + FCP_KDLS * dy1;
  }
  else
  {
  y1 = FCP_SETPTLS - qLS;
  dy1 = 0 - dqLS;
  uLS = FCP_KPLS * y1 + FCP_KDLS * dy1;
  }
  // Cap torques at +- 15
  torques.torqueA = 0 * abs_max(uLA - (uLS / 2), MAX_TORQUE);
  torques.torqueB = 0 * abs_max(uLA + (uLS / 2), MAX_TORQUE);
  */

  float des_mtr_angA = PI/2. - PI + acos( 0.9239 );
  float des_mtr_angB = PI/2. + PI - acos( 0.9239 ); 

  output->motor_torqueA = abs_max(TEST_CONTROLLER_DATA(data)->flight_motor_gain * (des_mtr_angA - input->motor_angleA) - 6. * input->motor_velocityA);
  output->motor_torqueB = abs_max(TEST_CONTROLLER_DATA(data)->flight_motor_gain * (des_mtr_angB - input->motor_angleB) - 6. * input->motor_velocityB);
}

void stance_state_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
  // Math from file
  float qLS = input->leg_angleB - input->leg_angleA;
  float dqLS = input->leg_velocityB - input->leg_velocityA;
  float vcm = (mT*(sin(input->body_angle+input->motor_angleB)*L2+sin(input->body_angle+input->motor_angleA)*L4+cos(input->body_angle)*ellycmT-sin(input->body_angle)*ellzcmT)+m1*(sin(input->body_angle+input->motor_angleB)*L2+sin(input->body_angle+input->motor_angleA)*L4+cos(input->body_angle+input->motor_angleA)*ellycm1-sin(input->body_angle+input->motor_angleA)*ellzcm1)+m2*(sin(input->body_angle+input->motor_angleB)*L2+sin(input->body_angle+input->motor_angleA)*L4+cos(input->body_angle+input->motor_angleB)*ellycm2-sin(input->body_angle+input->motor_angleB)*ellzcm2)+m3*(sin(input->body_angle+input->motor_angleB)*L2+sin(input->body_angle+input->motor_angleA)*L4-sin(input->body_angle+input->motor_angleA)*L1+cos(input->body_angle+input->motor_angleB)*ellycm3-sin(input->body_angle+input->motor_angleB)*ellzcm3)+m4*(sin(input->body_angle+input->motor_angleA)*L4+cos(input->body_angle+input->motor_angleA)*ellycm4-sin(input->body_angle+input->motor_angleA)*ellzcm4))/(mT+m1+m2+m3+m4)*input->body_ang_vel+(mT*sin(input->body_angle+input->motor_angleA)*L4+m1*(sin(input->body_angle+input->motor_angleA)*L4+cos(input->body_angle+input->motor_angleA)*ellycm1-sin(input->body_angle+input->motor_angleA)*ellzcm1)+m2*sin(input->body_angle+input->motor_angleA)*L4+m3*(sin(input->body_angle+input->motor_angleA)*L4-sin(input->body_angle+input->motor_angleA)*L1)+m4*(sin(input->body_angle+input->motor_angleA)*L4+cos(input->body_angle+input->motor_angleA)*ellycm4-sin(input->body_angle+input->motor_angleA)*ellzcm4))/(mT+m1+m2+m3+m4)*input->motor_velocityA+(mT*sin(input->body_angle+input->motor_angleB)*L2+m1*sin(input->body_angle+input->motor_angleB)*L2+m2*(sin(input->body_angle+input->motor_angleB)*L2+cos(input->body_angle+input->motor_angleB)*ellycm2-sin(input->body_angle+input->motor_angleB)*ellzcm2)+m3*(sin(input->body_angle+input->motor_angleB)*L2+cos(input->body_angle+input->motor_angleB)*ellycm3-sin(input->body_angle+input->motor_angleB)*ellzcm3))/(mT+m1+m2+m3+m4)*input->motor_velocityB;
  float qT = input->body_angle;
  float q1 = input->motor_angleA;
  float q2 = input->motor_angleB;
  float qgr1 = input->leg_angleA;
  float qgr2 = input->leg_angleB;
  float dqT = input->body_ang_vel;
  float dq1 = input->motor_velocityA;
  float dq2 = input->motor_velocityB;
  float dqgr1 = input->leg_velocityA;
  float dqgr2 = input->leg_velocityB;
  float PE = g*(mT*(-cos(qT+q2)*L2-cos(qT+q1)*L4+sin(qT)*ellycmT+cos(qT)*ellzcmT)+m1*(-cos(qT+q2)*L2-cos(qT+q1)*L4+sin(qT+q1)*ellycm1+cos(qT+q1)*ellzcm1)+m2*(-cos(qT+q2)*L2-cos(qT+q1)*L4+sin(qT+q2)*ellycm2+cos(qT+q2)*ellzcm2)+m3*(-cos(qT+q2)*L2-cos(qT+q1)*L4+cos(qT+q1)*L1+sin(qT+q2)*ellycm3+cos(qT+q2)*ellzcm3)+m4*(-cos(qT+q1)*L4+sin(qT+q1)*ellycm4+cos(qT+q1)*ellzcm4))+1.0/2.0*K1*((q1-qgr1)*(q1-qgr1))+1.0/2.0*K2*((q2-qgr2)*(q2-qgr2));
  float MapleGenVar3 = Jcm1*dqT*dq1+Jcm2*dqT*dq2+Jcm3*dqT*dq2+m2*dq2*dq2*ellycm2*ellycm2/2.0+m3*dqT*dqT*L1*L1/2.0+m1*dqT*dqT*ellzcm1*ellzcm1/2.0+m4*dq1*dq1*ellycm4*ellycm4/2.0+m3*dq2*dq2*ellzcm3*ellzcm3/2.0+m2*dqT*dqT*ellycm2*ellycm2/2.0+m1*dq1*dq1*ellzcm1*ellzcm1/2.0+m1*dqT*dqT*ellycm1*ellycm1/2.0+m1*dq1*dq1*ellycm1*ellycm1/2.0+m1*dqT*ellzcm1*ellzcm1*dq1+m1*dqT*ellycm1*ellycm1*dq1+m2*dqT*ellycm2*ellycm2*dq2+m2*dqT*ellzcm2*ellzcm2*dq2+m3*dqT*L1*L1*dq1+m3*dqT*ellzcm3*ellzcm3*dq2+m4*dqT*ellzcm4*ellzcm4*dq1+m3*dqT*ellycm3*ellycm3*dq2+m4*dqT*ellycm4*ellycm4*dq1+m3*L1*L1*dq1*dq1/2.0+m4*dq1*dq1*ellzcm4*ellzcm4/2.0+m4*dqT*dqT*ellycm4*ellycm4/2.0+m4*dqT*dqT*ellzcm4*ellzcm4/2.0+mT*dqT*dqT*ellycmT*ellycmT/2.0;
  float MapleGenVar4 = mT*dqT*dqT*ellzcmT*ellzcmT/2.0+m2*dqT*dqT*ellzcm2*ellzcm2/2.0+m2*dq2*dq2*ellzcm2*ellzcm2/2.0+m3*dqT*dqT*ellycm3*ellycm3/2.0+m3*dq2*dq2*ellycm3*ellycm3/2.0+m3*dqT*dqT*ellzcm3*ellzcm3/2.0+Jcm4*dqT*dq1+m3*dqT*sin(qT+q2)*ellycm3*cos(qT+q1)*L1*dq1+m3*dqT*cos(qT+q2)*ellzcm3*cos(qT+q1)*L1*dq1+m3*cos(qT+q1)*L1*dq1*dq2*sin(qT+q2)*ellycm3+m3*cos(qT+q1)*L1*dq1*dq2*cos(qT+q2)*ellzcm3+m3*dqT*dqT*cos(qT+q1)*L1*sin(qT+q2)*ellycm3+m3*dqT*cos(qT+q1)*L1*dq2*cos(qT+q2)*ellzcm3;
  float MapleGenVar2 = MapleGenVar4+m3*dqT*dqT*sin(qT+q1)*L1*sin(qT+q2)*ellzcm3-m3*dqT*sin(qT+q1)*L1*dq2*cos(qT+q2)*ellycm3+m3*dqT*sin(qT+q1)*L1*dq2*sin(qT+q2)*ellzcm3-m3*dqT*cos(qT+q2)*ellycm3*sin(qT+q1)*L1*dq1+m3*dqT*sin(qT+q2)*ellzcm3*sin(qT+q1)*L1*dq1-m3*sin(qT+q1)*L1*dq1*dq2*cos(qT+q2)*ellycm3+m3*sin(qT+q1)*L1*dq1*dq2*sin(qT+q2)*ellzcm3-m3*dqT*dqT*sin(qT+q1)*L1*cos(qT+q2)*ellycm3+Jcm4*dq1*dq1/2.0+Jcm4*dqT*dqT/2.0+Jrotor1*pow(dqT+dqgr1*R1,2.0)/2.0+JcmT*dqT*dqT/2.0+Jgear1*pow(dqT+dqgr1,2.0)/2.0+MapleGenVar3;
  MapleGenVar4 = Jcm2*dq2*dq2/2.0+Jcm2*dqT*dqT/2.0+Jcm3*dqT*dqT/2.0+Jcm3*dq2*dq2/2.0+Jrotor2*pow(dqT+dqgr2*R2,2.0)/2.0+Jcm1*dqT*dqT/2.0+Jcm1*dq1*dq1/2.0+Jgear2*pow(dqT+dqgr2,2.0)/2.0+m3*dqT*dqT*cos(qT+q1)*L1*cos(qT+q2)*ellzcm3+m3*dqT*cos(qT+q1)*L1*dq2*sin(qT+q2)*ellycm3+m1*dqT*sin(qT+q2)*L2*dq1*cos(qT+q1)*ellycm1-m1*dqT*sin(qT+q1)*ellycm1*cos(qT+q2)*L2*dq2+m1*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4;
  float MapleGenVar5 = MapleGenVar4-m1*dqT*cos(qT+q2)*L2*dq1*sin(qT+q1)*ellycm1-m1*dqT*cos(qT+q2)*L2*dq1*cos(qT+q1)*ellzcm1+m1*dqT*cos(qT+q1)*L4*cos(qT+q2)*L2*dq2+m1*cos(qT+q1)*L4*dq1*cos(qT+q2)*L2*dq2-m1*dqT*cos(qT+q1)*ellzcm1*cos(qT+q2)*L2*dq2-m1*dq1*sin(qT+q1)*ellycm1*cos(qT+q2)*L2*dq2;
  MapleGenVar3 = MapleGenVar5-m1*dqT*dqT*cos(qT+q2)*L2*sin(qT+q1)*ellycm1-m1*dq1*cos(qT+q1)*ellzcm1*cos(qT+q2)*L2*dq2-m1*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*ellzcm1+m1*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4*dq1+m1*dqT*sin(qT+q1)*L4*sin(qT+q2)*L2*dq2+m2*sin(qT+q1)*L4*dq1*sin(qT+q2)*L2*dq2+m2*dqT*sin(qT+q1)*L4*dq2*cos(qT+q2)*ellycm2;
  MapleGenVar4 = MapleGenVar3-m1*dqT*sin(qT+q2)*L2*dq1*sin(qT+q1)*ellzcm1+MapleGenVar2+mT*dqT*dqT*sin(qT+q2)*L2*cos(qT)*ellycmT+mT*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4+mT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4*dq1-mT*dqT*cos(qT)*ellzcmT*cos(qT+q2)*L2*dq2-2.0*m3*dqT*L2*dq2*ellzcm3+mT*dqT*dqT*sin(qT+q1)*L4*cos(qT)*ellycmT-mT*dqT*dqT*cos(qT+q1)*L4*cos(qT)*ellzcmT+mT*dqT*sin(qT+q1)*L4*sin(qT+q2)*L2*dq2+mT*dqT*cos(qT)*ellycmT*sin(qT+q2)*L2*dq2+mT*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4+mT*sin(qT+q1)*L4*dq1*sin(qT+q2)*L2*dq2;
  float MapleGenVar1 = MapleGenVar4+mT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4*dq1-mT*dqT*dqT*cos(qT+q2)*L2*sin(qT)*ellycmT+mT*cos(qT+q1)*L4*dq1*cos(qT+q2)*L2*dq2+mT*dqT*cos(qT)*ellycmT*sin(qT+q1)*L4*dq1-mT*dqT*dqT*cos(qT+q2)*L2*cos(qT)*ellzcmT-mT*dqT*sin(qT)*ellzcmT*sin(qT+q1)*L4*dq1-mT*dqT*dqT*cos(qT+q1)*L4*sin(qT)*ellycmT-mT*dqT*sin(qT)*ellycmT*cos(qT+q1)*L4*dq1+m3*L2*L2*dq2*dq2/2.0+mT*dqT*cos(qT+q1)*L4*cos(qT+q2)*L2*dq2-mT*dqT*cos(qT)*ellzcmT*cos(qT+q1)*L4*dq1-2.0*m2*dqT*L2*dq2*ellzcm2-m3*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L1-m3*dqT*cos(qT+q1)*L4*dq2*sin(qT+q2)*ellycm3;
  MapleGenVar4 = MapleGenVar1-m3*dqT*cos(qT+q1)*L4*dq2*cos(qT+q2)*ellzcm3-m3*dqT*cos(qT+q1)*L1*cos(qT+q2)*L2*dq2-m3*dqT*sin(qT+q2)*ellycm3*cos(qT+q1)*L4*dq1-m3*dqT*cos(qT+q2)*ellzcm3*cos(qT+q1)*L4*dq1+mT*dqT*L4*L4*dq1+mT*dqT*L2*L2*dq2-m4*L4*dq1*dq1*ellzcm4-2.0*m1*dqT*L4*dq1*ellzcm1-mT*dqT*sin(qT)*ellycmT*cos(qT+q2)*L2*dq2-mT*dqT*dqT*sin(qT+q1)*L4*sin(qT)*ellzcmT-mT*dqT*dqT*sin(qT+q2)*L2*sin(qT)*ellzcmT-mT*dqT*sin(qT)*ellzcmT*sin(qT+q2)*L2*dq2;
  MapleGenVar3 = MapleGenVar4-2.0*m4*dqT*L4*dq1*ellzcm4+m1*L4*L4*dq1*dq1/2.0+m1*L2*L2*dq2*dq2/2.0-m3*cos(qT+q1)*L4*dq1*dq2*sin(qT+q2)*ellycm3-m3*cos(qT+q1)*L4*dq1*dq2*cos(qT+q2)*ellzcm3-m3*dq1*cos(qT+q1)*L1*cos(qT+q2)*L2*dq2-m3*dqT*cos(qT+q2)*L2*dq1*cos(qT+q1)*L1-m3*dqT*dqT*cos(qT+q1)*L4*sin(qT+q2)*ellycm3+m3*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4-m2*dqT*sin(qT+q1)*L4*dq2*sin(qT+q2)*ellzcm2+m2*dqT*cos(qT+q2)*ellycm2*sin(qT+q1)*L4*dq1-m1*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*ellzcm1+m2*dqT*sin(qT+q1)*L4*sin(qT+q2)*L2*dq2;
  MapleGenVar4 = MapleGenVar3+m2*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4*dq1+m2*dqT*dqT*sin(qT+q1)*L4*cos(qT+q2)*ellycm2-m3*L2*dq2*dq2*ellzcm3-m3*L4*dq1*dq1*L1-m3*dqT*dqT*L4*L1-m3*dqT*dqT*L2*ellzcm3+m3*dqT*L2*L2*dq2+m3*dqT*L4*L4*dq1-m4*dqT*dqT*L4*ellzcm4+m4*dqT*L4*L4*dq1+m1*dqT*L2*L2*dq2+m1*dqT*L4*L4*dq1;
  MapleGenVar2 = MapleGenVar4-m1*L4*dq1*dq1*ellzcm1-m1*dqT*dqT*L4*ellzcm1+m2*dqT*L4*L4*dq1+m2*dqT*L2*L2*dq2-m2*dqT*dqT*L2*ellzcm2-m2*L2*dq2*dq2*ellzcm2+m2*cos(qT+q1)*L4*dq1*cos(qT+q2)*L2*dq2+m2*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4+m2*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4-m2*dqT*cos(qT+q1)*L4*dq2*sin(qT+q2)*ellycm2-m2*dqT*cos(qT+q1)*L4*dq2*cos(qT+q2)*ellzcm2-m2*dqT*sin(qT+q2)*ellycm2*cos(qT+q1)*L4*dq1-m2*cos(qT+q1)*L4*dq1*dq2*sin(qT+q2)*ellycm2-m2*cos(qT+q1)*L4*dq1*dq2*cos(qT+q2)*ellzcm2;
  MapleGenVar4 = MapleGenVar2+m3*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4-m3*dqT*dqT*cos(qT+q1)*L4*cos(qT+q2)*ellzcm3+m3*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4*dq1+m3*dqT*dqT*L2*L2/2.0+m3*dqT*dqT*L4*L4/2.0+m3*L4*L4*dq1*dq1/2.0+mT*dqT*dqT*L2*L2/2.0+mT*L2*L2*dq2*dq2/2.0+mT*dqT*dqT*L4*L4/2.0+mT*L4*L4*dq1*dq1/2.0+m1*dqT*dqT*L2*L2/2.0+m2*L2*L2*dq2*dq2/2.0;
  MapleGenVar3 = MapleGenVar4+m2*L4*L4*dq1*dq1/2.0+m2*dqT*dqT*L4*L4/2.0+m2*dqT*dqT*L2*L2/2.0+m4*L4*L4*dq1*dq1/2.0+m4*dqT*dqT*L4*L4/2.0+m1*dqT*dqT*L4*L4/2.0+m1*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4-m2*dqT*dqT*sin(qT+q1)*L4*sin(qT+q2)*ellzcm2+m1*dqT*cos(qT+q1)*ellycm1*sin(qT+q2)*L2*dq2-m1*dqT*sin(qT+q1)*ellzcm1*sin(qT+q2)*L2*dq2+m1*dq1*cos(qT+q1)*ellycm1*sin(qT+q2)*L2*dq2-m1*dq1*sin(qT+q1)*ellzcm1*sin(qT+q2)*L2*dq2+m1*dqT*dqT*sin(qT+q2)*L2*cos(qT+q1)*ellycm1+m1*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4*dq1;
  MapleGenVar5 = MapleGenVar3+m1*sin(qT+q1)*L4*dq1*sin(qT+q2)*L2*dq2+m2*dqT*cos(qT+q1)*L4*cos(qT+q2)*L2*dq2-m2*dqT*dqT*cos(qT+q1)*L4*sin(qT+q2)*ellycm2-m2*dqT*sin(qT+q2)*ellzcm2*sin(qT+q1)*L4*dq1+m2*sin(qT+q1)*L4*dq1*dq2*cos(qT+q2)*ellycm2+m2*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4*dq1;
  MapleGenVar4 = MapleGenVar5-m2*sin(qT+q1)*L4*dq1*dq2*sin(qT+q2)*ellzcm2-m2*dqT*cos(qT+q2)*ellzcm2*cos(qT+q1)*L4*dq1-m2*dqT*dqT*cos(qT+q1)*L4*cos(qT+q2)*ellzcm2+m3*dqT*dqT*sin(qT+q1)*L4*cos(qT+q2)*ellycm3+m3*dqT*sin(qT+q1)*L4*sin(qT+q2)*L2*dq2+m3*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4*dq1+m3*sin(qT+q1)*L4*dq1*sin(qT+q2)*L2*dq2;
  MapleGenVar5 = MapleGenVar4-m3*dqT*sin(qT+q2)*L2*dq1*sin(qT+q1)*L1-m3*dqT*dqT*sin(qT+q1)*L4*sin(qT+q2)*ellzcm3-m3*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L1+m3*dqT*sin(qT+q1)*L4*dq2*cos(qT+q2)*ellycm3-m3*dqT*sin(qT+q1)*L4*dq2*sin(qT+q2)*ellzcm3-m3*dqT*sin(qT+q1)*L1*sin(qT+q2)*L2*dq2;
  float KE;
  KE = MapleGenVar5+m3*dqT*cos(qT+q2)*ellycm3*sin(qT+q1)*L4*dq1-m3*dqT*sin(qT+q2)*ellzcm3*sin(qT+q1)*L4*dq1+m3*sin(qT+q1)*L4*dq1*dq2*cos(qT+q2)*ellycm3-m3*sin(qT+q1)*L4*dq1*dq2*sin(qT+q2)*ellzcm3-m3*dq1*sin(qT+q1)*L1*sin(qT+q2)*L2*dq2+m3*dqT*cos(qT+q1)*L4*cos(qT+q2)*L2*dq2+m3*cos(qT+q1)*L4*dq1*cos(qT+q2)*L2*dq2-2.0*m3*dqT*L4*dq1*L1;
  float Etot = PE + KE;
  float uLSpower, uLSspring, uLS;
  float y1 = SCP_SETPTLS - qLS;
  float dy1 = 0 - dqLS;

  // Controller choices
  switch(STANCE_CONTROLLER)
    {
    case 1:
      uLSpower = -75 * dqLS * (Etot - SCP_EDES);
      uLSspring = SCP_KPLS * y1 + SCP_KDLS * dy1;
      uLS = uLSpower + uLSspring;
      break;
    case 2:
      if(vcm < -0.5/2)
	{
	  uLSspring = (1.0/ALPHA) * SCP_KPLS * y1 + (1.0/ALPHA) * SCP_KDLS * dy1;
	  uLSpower = 0;
	}
      else
	{
	  uLSpower = -50 * dqLS * (Etot - SCP_EDES);
	  uLSspring = ALPHA * SCP_KPLS * y1;
	}
      uLS = uLSpower + uLSspring;
      break;
    case 3:
      if(vcm < -0.5)
	{
	  uLS = (1.0/ALPHA) * SCP_KPLS * y1 + ALPHA * SCP_KDLS * dy1;
	}
      else
	{
	  uLS =  ALPHA * (SCP_KPLS * y1 + SCP_KDLS * dy1);
	}
      break;
    default:
      PRINT_MSG("Stance controller selected improperly:");
      break;
    }
  float y2 = SCP_SETPTTDA - input->body_angle;
  float dy2 = 0 - input->body_ang_vel;
  float uTorso = SCP_KPTDA * y2 + SCP_KDTDA * dy2;
  // Cap torques at +- 15
  output->motor_torqueA = TEST_CONTROLLER_DATA(data)->stance_motor_gain * abs_max(-uTorso - (uLS / 2), MAX_TORQUE);
  output->motor_torqueB = TEST_CONTROLLER_DATA(data)->stance_motor_gain * abs_max(-uTorso + (uLS / 2), MAX_TORQUE);
}

void abs_max(float *number, float max)
{
  if(ABS(number) > max)
    {
      number = (number > 0)? max : -max;
    }
}
