// Ported by: Colan Dray & Alireza Ramezani
// MATLAB code by: Jessy Grizzle

#include <iostream>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265
#define g 9.81
#define MAX_TORQUE 15.0

double FCP_SETPTLS = 0.7854;
const double FCP_KPLS = 400.0;
const double FCP_KDLS = 50.4077;
const double FCP_KPTDA = 750.0;
const double FCP_KDTDA = 50.0;
const double FCP_SETPTTDA = 0.1230;
const double FCP_SETPTTDA_NOM = 0.0524;
const double SCP_KPLS = 600.0;
const double SCP_KDLS = 167.8620;
const double SCP_SETPTLS = 0.2618;
const double SCP_KPTDA = 150.0;
const double SCP_KDTDA = 26.6597;
const double SCP_SETPTTDA = -0.0175;
const double SCP_EDES = 273.0;
const double ALPHA = 3.0;
const double L1 = 0.45;
const double L2 = 0.5;
const double L3 = 0.5;
const double L4 = 0.5;
const double m1 = 0.66149;
const double m2 = 0.68292;
const double m3 = 0.19126;
const double m4 = 0.42493;
const double mT = 44.0 / 2.0;
const double R1 = 20;
const double R2 = R1;
const double K1 = 1200.0 / 0.5;
const double K2 = K1;
const double Jcm1 = 0.01910;
const double Jcm2 = 0.02116;
const double Jcm3 = 0.00633;
const double Jcm4 = 0.01243;
const double JcmT = 1.37;
const double Jgear1 = 0.0025;
const double Jgear2 = Jgear1;
const double Jrotor1 = 0.00286;
const double Jrotor2 = Jrotor1;
const double ellzcm1 = 0.16957;
const double ellzcm2 = 0.18626;
const double ellzcm3 = 0.24997;
const double ellzcm4 = 0.23832;
const double ellycm1 = 0.04566;
const double ellycm2 = -0.02624;
const double ellycm3 = 0.0;
const double ellycm4 = 0.0;
const double ellzcmTa  = 0.01;
const double ellycmT = 0.0;
const double mBatteryPack = 6.7;
const double ellzBatteryBack = -0.09;
const double ellzcmT = ellzcmTa -2*ellzBatteryBack *mBatteryPack/(mT);
const int STANCE_CONTROLLER = 2;

struct TorqueOutputs
{
  double torqueA;
  double torqueB;
};

struct AngleInputs
{
  double torso;
  double motor1;
  double motor2;
  double gear1;
  double gear2;
  double dtorso;
  double dmotor1;
  double dmotor2;
  double dgear1;
  double dgear2;
};

struct PositionInputs
{
  double Y;
  double Z;
  double dY;
  double dZ;
};

void flight_state_controller(AngleInputs angles, PositionInputs positions, TorqueOutputs& output);
void stance_state_controller(AngleInputs angles, TorqueOutputs& output);
double abs_max(double var, double max);


extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
  TEST_CONTROLLER_STATE(state)->in_flight = true;
  output->motor_torqueA = output->motor_torqueB = 0.0;

  PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
  AngleInputs angles;
  PositionInputs positions;
  TorqueOutputs outputs;

  // Get sensor data
  angles.torso = input->body_angle;
  angles.motor1 = input->motor_angleA;
  angles.motor2 = input->motor_angleB;
  angles.gear1 = input->leg_angleA;
  angles.gear2 = input->leg_angleB;
  angles.dtorso = input->body_velocity;
  angles.dmotor1 = input->motor_velocityA;
  angles.dmotor2 = input->motor_velocityB;
  angles.dgear1 = input->leg_velocityA;
  angles.dgear2 = input->leg_velocityB;
  //positions.Y = 0;
  positions.Z = input->height;
  //positions.dY = input->horizontal_velocity;
  positions.dZ = input->vertical_velocity;

  // Choose a controller based on state, then check to see if we need to change state
  if(TEST_CONTROLLER_STATE(state)->in_flight)
    {
      flight_state_controller(angles, positions, outputs);
      if ( (abs(input->motor_angleA - input->leg_angleA) > FLIGHT_THRESHOLD) ||
	   (abs(input->motor_angleB - input->leg_angleB) > FLIGHT_THRESHOLD) )
	{
	  PRINT_MSG("Test controller status: LANDED");
	  TEST_CONTROLLER_STATE(state)->in_flight = false;
	}
    }
  else
    {
      stance_state_controller(angles, outputs);
      if ( ( abs(spring_defA) < STANCE_THRESHOLD ) &&
	   ( abs(spring_defB) < STANCE_THRESHOLD ) )
	{
	  PRINT_MSG("Test controller status: TAKEOFF");
	  RAIBERT_CONTROLLER_STATE(state)->in_flight = true;
	}
    }

  /*
  // qT=q_minus(1);

  double ref_speed = 1.3;
  double error_speed = ref_speed - vcm(1);
  if(error_speed > 0.5)
  {
  FCP_SETPTTDA = -1.0 * PI / 180;
  }
  else if(error_speed < -.3)
  {
  FCP_SETPTTDA = FCP_SETPTTDA_NOM - 12.0 * error_speed * PI / 180;
  }
  else
  {
  FCP_SETPTTDA = ((speed < 0)? -1 : 1) * FCP_SETPTTDA_NOM;
  }
  end
  */

  // Send output torques to motors
  output->motor_torqueA = outputs.torqueA;
  output->motor_torqueB = outputs.torqueB;
}

extern void takedown_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, Controlleata *data)
{
  output->motor_torqueA = output->motor_torqueB = 0.0;

  PRINT_MSG("Test Controller Stopped.\n");
}

// Name: flight_state_controller
// Description: Controller for flight state
// Parameters:
//  angles: Input angles
//  positions: Input positions
//  &output: Pair of output torques
// Output: void
void flight_state_controller(AngleInputs angles, PositionInputs positions, TorqueOutputs& output)
{
  // Math from file
  double vHip2 = positions.dZ;
  double p42 = positions.Z + cos(angles.torso + angles.motor2) * L2 + cos(angles.torso + angles.motor1) * L4;
  double qLA = (angles.motor1 + angles.motor2) / 2;
  double dqLA = (angles.dmotor1 + angles.dmotor2) / 2;
  double qLS = angles.gear2 - angles.gear1;
  double dqLS = angles.dgear2 - angles.dgear1;
  double qLA_abs = qLA + angles.torso;
  double qTDA = qLA_abs - PI;
  double dqLA_abs = dqLA + angles.dtorso;
  double dqTDA = dqLA_abs;
  double y2 = FCP_SETPTTDA - qTDA;
  double dy2 = 0 - dqTDA;
  double uLA = FCP_KPTDA * y2 + FCP_KDTDA * dy2;
  double y1, dy1, uLS;
  if( (vHip2 > 0) && (p42 < 0.05) )
    {
      FCP_SETPTLS += 10.0;
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
  output.torqueA = abs_max(uLA - (uLS / 2), MAX_TORQUE);
  output.torqueB = abs_max(uLA + (uLS / 2), MAX_TORQUE);
}

// Name: stance_state_controller
// Description: Controller for stance state
// Parameters:
//  angles: Input angles
//  &output: Pair of output torques
// Output: void
void stance_state_controller(AngleInputs angles, TorqueOutputs& output)
{
  // Math from file
  double qLS = angles.gear2 - angles.gear1;
  double dqLS = angles.dgear2 - angles.dgear1;
  double vcm = (mT*(sin(angles.torso+angles.motor2)*L2+sin(angles.torso+angles.motor1)*L4+cos(angles.torso)*ellycmT-sin(angles.torso)*ellzcmT)+m1*(sin(angles.torso+angles.motor2)*L2+sin(angles.torso+angles.motor1)*L4+cos(angles.torso+angles.motor1)*ellycm1-sin(angles.torso+angles.motor1)*ellzcm1)+m2*(sin(angles.torso+angles.motor2)*L2+sin(angles.torso+angles.motor1)*L4+cos(angles.torso+angles.motor2)*ellycm2-sin(angles.torso+angles.motor2)*ellzcm2)+m3*(sin(angles.torso+angles.motor2)*L2+sin(angles.torso+angles.motor1)*L4-sin(angles.torso+angles.motor1)*L1+cos(angles.torso+angles.motor2)*ellycm3-sin(angles.torso+angles.motor2)*ellzcm3)+m4*(sin(angles.torso+angles.motor1)*L4+cos(angles.torso+angles.motor1)*ellycm4-sin(angles.torso+angles.motor1)*ellzcm4))/(mT+m1+m2+m3+m4)*angles.dtorso+(mT*sin(angles.torso+angles.motor1)*L4+m1*(sin(angles.torso+angles.motor1)*L4+cos(angles.torso+angles.motor1)*ellycm1-sin(angles.torso+angles.motor1)*ellzcm1)+m2*sin(angles.torso+angles.motor1)*L4+m3*(sin(angles.torso+angles.motor1)*L4-sin(angles.torso+angles.motor1)*L1)+m4*(sin(angles.torso+angles.motor1)*L4+cos(angles.torso+angles.motor1)*ellycm4-sin(angles.torso+angles.motor1)*ellzcm4))/(mT+m1+m2+m3+m4)*angles.dmotor1+(mT*sin(angles.torso+angles.motor2)*L2+m1*sin(angles.torso+angles.motor2)*L2+m2*(sin(angles.torso+angles.motor2)*L2+cos(angles.torso+angles.motor2)*ellycm2-sin(angles.torso+angles.motor2)*ellzcm2)+m3*(sin(angles.torso+angles.motor2)*L2+cos(angles.torso+angles.motor2)*ellycm3-sin(angles.torso+angles.motor2)*ellzcm3))/(mT+m1+m2+m3+m4)*angles.dmotor2;
  double qT = angles.torso;
  double q1 = angles.motor1;
  double q2 = angles.motor2;
  double qgr1 = angles.gear1;
  double qgr2 = angles.gear2;
  double dqT = angles.dtorso;
  double dq1 = angles.dmotor1;
  double dq2 = angles.dmotor2;
  double dqgr1 = angles.dgear1;
  double dqgr2 = angles.dgear2;
  double PE = g*(mT*(-cos(qT+q2)*L2-cos(qT+q1)*L4+sin(qT)*ellycmT+cos(qT)*ellzcmT)+m1*(-cos(qT+q2)*L2-cos(qT+q1)*L4+sin(qT+q1)*ellycm1+cos(qT+q1)*ellzcm1)+m2*(-cos(qT+q2)*L2-cos(qT+q1)*L4+sin(qT+q2)*ellycm2+cos(qT+q2)*ellzcm2)+m3*(-cos(qT+q2)*L2-cos(qT+q1)*L4+cos(qT+q1)*L1+sin(qT+q2)*ellycm3+cos(qT+q2)*ellzcm3)+m4*(-cos(qT+q1)*L4+sin(qT+q1)*ellycm4+cos(qT+q1)*ellzcm4))+1.0/2.0*K1*((q1-qgr1)*(q1-qgr1))+1.0/2.0*K2*((q2-qgr2)*(q2-qgr2));
  double MapleGenVar3 = Jcm1*dqT*dq1+Jcm2*dqT*dq2+Jcm3*dqT*dq2+m2*dq2*dq2*ellycm2*ellycm2/2.0+m3*dqT*dqT*L1*L1/2.0+m1*dqT*dqT*ellzcm1*ellzcm1/2.0+m4*dq1*dq1*ellycm4*ellycm4/2.0+m3*dq2*dq2*ellzcm3*ellzcm3/2.0+m2*dqT*dqT*ellycm2*ellycm2/2.0+m1*dq1*dq1*ellzcm1*ellzcm1/2.0+m1*dqT*dqT*ellycm1*ellycm1/2.0+m1*dq1*dq1*ellycm1*ellycm1/2.0+m1*dqT*ellzcm1*ellzcm1*dq1+m1*dqT*ellycm1*ellycm1*dq1+m2*dqT*ellycm2*ellycm2*dq2+m2*dqT*ellzcm2*ellzcm2*dq2+m3*dqT*L1*L1*dq1+m3*dqT*ellzcm3*ellzcm3*dq2+m4*dqT*ellzcm4*ellzcm4*dq1+m3*dqT*ellycm3*ellycm3*dq2+m4*dqT*ellycm4*ellycm4*dq1+m3*L1*L1*dq1*dq1/2.0+m4*dq1*dq1*ellzcm4*ellzcm4/2.0+m4*dqT*dqT*ellycm4*ellycm4/2.0+m4*dqT*dqT*ellzcm4*ellzcm4/2.0+mT*dqT*dqT*ellycmT*ellycmT/2.0;
  double MapleGenVar4 = mT*dqT*dqT*ellzcmT*ellzcmT/2.0+m2*dqT*dqT*ellzcm2*ellzcm2/2.0+m2*dq2*dq2*ellzcm2*ellzcm2/2.0+m3*dqT*dqT*ellycm3*ellycm3/2.0+m3*dq2*dq2*ellycm3*ellycm3/2.0+m3*dqT*dqT*ellzcm3*ellzcm3/2.0+Jcm4*dqT*dq1+m3*dqT*sin(qT+q2)*ellycm3*cos(qT+q1)*L1*dq1+m3*dqT*cos(qT+q2)*ellzcm3*cos(qT+q1)*L1*dq1+m3*cos(qT+q1)*L1*dq1*dq2*sin(qT+q2)*ellycm3+m3*cos(qT+q1)*L1*dq1*dq2*cos(qT+q2)*ellzcm3+m3*dqT*dqT*cos(qT+q1)*L1*sin(qT+q2)*ellycm3+m3*dqT*cos(qT+q1)*L1*dq2*cos(qT+q2)*ellzcm3;
  double MapleGenVar2 = MapleGenVar4+m3*dqT*dqT*sin(qT+q1)*L1*sin(qT+q2)*ellzcm3-m3*dqT*sin(qT+q1)*L1*dq2*cos(qT+q2)*ellycm3+m3*dqT*sin(qT+q1)*L1*dq2*sin(qT+q2)*ellzcm3-m3*dqT*cos(qT+q2)*ellycm3*sin(qT+q1)*L1*dq1+m3*dqT*sin(qT+q2)*ellzcm3*sin(qT+q1)*L1*dq1-m3*sin(qT+q1)*L1*dq1*dq2*cos(qT+q2)*ellycm3+m3*sin(qT+q1)*L1*dq1*dq2*sin(qT+q2)*ellzcm3-m3*dqT*dqT*sin(qT+q1)*L1*cos(qT+q2)*ellycm3+Jcm4*dq1*dq1/2.0+Jcm4*dqT*dqT/2.0+Jrotor1*pow(dqT+dqgr1*R1,2.0)/2.0+JcmT*dqT*dqT/2.0+Jgear1*pow(dqT+dqgr1,2.0)/2.0+MapleGenVar3;
  MapleGenVar4 = Jcm2*dq2*dq2/2.0+Jcm2*dqT*dqT/2.0+Jcm3*dqT*dqT/2.0+Jcm3*dq2*dq2/2.0+Jrotor2*pow(dqT+dqgr2*R2,2.0)/2.0+Jcm1*dqT*dqT/2.0+Jcm1*dq1*dq1/2.0+Jgear2*pow(dqT+dqgr2,2.0)/2.0+m3*dqT*dqT*cos(qT+q1)*L1*cos(qT+q2)*ellzcm3+m3*dqT*cos(qT+q1)*L1*dq2*sin(qT+q2)*ellycm3+m1*dqT*sin(qT+q2)*L2*dq1*cos(qT+q1)*ellycm1-m1*dqT*sin(qT+q1)*ellycm1*cos(qT+q2)*L2*dq2+m1*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4;
  double MapleGenVar5 = MapleGenVar4-m1*dqT*cos(qT+q2)*L2*dq1*sin(qT+q1)*ellycm1-m1*dqT*cos(qT+q2)*L2*dq1*cos(qT+q1)*ellzcm1+m1*dqT*cos(qT+q1)*L4*cos(qT+q2)*L2*dq2+m1*cos(qT+q1)*L4*dq1*cos(qT+q2)*L2*dq2-m1*dqT*cos(qT+q1)*ellzcm1*cos(qT+q2)*L2*dq2-m1*dq1*sin(qT+q1)*ellycm1*cos(qT+q2)*L2*dq2;
  MapleGenVar3 = MapleGenVar5-m1*dqT*dqT*cos(qT+q2)*L2*sin(qT+q1)*ellycm1-m1*dq1*cos(qT+q1)*ellzcm1*cos(qT+q2)*L2*dq2-m1*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*ellzcm1+m1*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4*dq1+m1*dqT*sin(qT+q1)*L4*sin(qT+q2)*L2*dq2+m2*sin(qT+q1)*L4*dq1*sin(qT+q2)*L2*dq2+m2*dqT*sin(qT+q1)*L4*dq2*cos(qT+q2)*ellycm2;
  MapleGenVar4 = MapleGenVar3-m1*dqT*sin(qT+q2)*L2*dq1*sin(qT+q1)*ellzcm1+MapleGenVar2+mT*dqT*dqT*sin(qT+q2)*L2*cos(qT)*ellycmT+mT*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4+mT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4*dq1-mT*dqT*cos(qT)*ellzcmT*cos(qT+q2)*L2*dq2-2.0*m3*dqT*L2*dq2*ellzcm3+mT*dqT*dqT*sin(qT+q1)*L4*cos(qT)*ellycmT-mT*dqT*dqT*cos(qT+q1)*L4*cos(qT)*ellzcmT+mT*dqT*sin(qT+q1)*L4*sin(qT+q2)*L2*dq2+mT*dqT*cos(qT)*ellycmT*sin(qT+q2)*L2*dq2+mT*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4+mT*sin(qT+q1)*L4*dq1*sin(qT+q2)*L2*dq2;
  double MapleGenVar1 = MapleGenVar4+mT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4*dq1-mT*dqT*dqT*cos(qT+q2)*L2*sin(qT)*ellycmT+mT*cos(qT+q1)*L4*dq1*cos(qT+q2)*L2*dq2+mT*dqT*cos(qT)*ellycmT*sin(qT+q1)*L4*dq1-mT*dqT*dqT*cos(qT+q2)*L2*cos(qT)*ellzcmT-mT*dqT*sin(qT)*ellzcmT*sin(qT+q1)*L4*dq1-mT*dqT*dqT*cos(qT+q1)*L4*sin(qT)*ellycmT-mT*dqT*sin(qT)*ellycmT*cos(qT+q1)*L4*dq1+m3*L2*L2*dq2*dq2/2.0+mT*dqT*cos(qT+q1)*L4*cos(qT+q2)*L2*dq2-mT*dqT*cos(qT)*ellzcmT*cos(qT+q1)*L4*dq1-2.0*m2*dqT*L2*dq2*ellzcm2-m3*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L1-m3*dqT*cos(qT+q1)*L4*dq2*sin(qT+q2)*ellycm3;
  MapleGenVar4 = MapleGenVar1-m3*dqT*cos(qT+q1)*L4*dq2*cos(qT+q2)*ellzcm3-m3*dqT*cos(qT+q1)*L1*cos(qT+q2)*L2*dq2-m3*dqT*sin(qT+q2)*ellycm3*cos(qT+q1)*L4*dq1-m3*dqT*cos(qT+q2)*ellzcm3*cos(qT+q1)*L4*dq1+mT*dqT*L4*L4*dq1+mT*dqT*L2*L2*dq2-m4*L4*dq1*dq1*ellzcm4-2.0*m1*dqT*L4*dq1*ellzcm1-mT*dqT*sin(qT)*ellycmT*cos(qT+q2)*L2*dq2-mT*dqT*dqT*sin(qT+q1)*L4*sin(qT)*ellzcmT-mT*dqT*dqT*sin(qT+q2)*L2*sin(qT)*ellzcmT-mT*dqT*sin(qT)*ellzcmT*sin(qT+q2)*L2*dq2;
  MapleGenVar3 = MapleGenVar4-2.0*m4*dqT*L4*dq1*ellzcm4+m1*L4*L4*dq1*dq1/2.0+m1*L2*L2*dq2*dq2/2.0-m3*cos(qT+q1)*L4*dq1*dq2*sin(qT+q2)*ellycm3-m3*cos(qT+q1)*L4*dq1*dq2*cos(qT+q2)*ellzcm3-m3*dq1*cos(qT+q1)*L1*cos(qT+q2)*L2*dq2-m3*dqT*cos(qT+q2)*L2*dq1*cos(qT+q1)*L1-m3*dqT*dqT*cos(qT+q1)*L4*sin(qT+q2)*ellycm3+m3*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4-m2*dqT*sin(qT+q1)*L4*dq2*sin(qT+q2)*ellzcm2+m2*dqT*cos(qT+q2)*ellycm2*sin(qT+q1)*L4*dq1-m1*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*ellzcm1+m2*dqT*sin(qT+q1)*L4*sin(qT+q2)*L2*dq2;
  MapleGenVar4 = MapleGenVar3+m2*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4*dq1+m2*dqT*dqT*sin(qT+q1)*L4*cos(qT+q2)*ellycm2-m3*L2*dq2*dq2*ellzcm3-m3*L4*dq1*dq1*L1-m3*dqT*dqT*L4*L1-m3*dqT*dqT*L2*ellzcm3+m3*dqT*L2*L2*dq2+m3*dqT*L4*L4*dq1-m4*dqT*dqT*L4*ellzcm4+m4*dqT*L4*L4*dq1+m1*dqT*L2*L2*dq2+m1*dqT*L4*L4*dq1;
  MapleGenVar2 = MapleGenVar4-m1*L4*dq1*dq1*ellzcm1-m1*dqT*dqT*L4*ellzcm1+m2*dqT*L4*L4*dq1+m2*dqT*L2*L2*dq2-m2*dqT*dqT*L2*ellzcm2-m2*L2*dq2*dq2*ellzcm2+m2*cos(qT+q1)*L4*dq1*cos(qT+q2)*L2*dq2+m2*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4+m2*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4-m2*dqT*cos(qT+q1)*L4*dq2*sin(qT+q2)*ellycm2-m2*dqT*cos(qT+q1)*L4*dq2*cos(qT+q2)*ellzcm2-m2*dqT*sin(qT+q2)*ellycm2*cos(qT+q1)*L4*dq1-m2*cos(qT+q1)*L4*dq1*dq2*sin(qT+q2)*ellycm2-m2*cos(qT+q1)*L4*dq1*dq2*cos(qT+q2)*ellzcm2;
  MapleGenVar4 = MapleGenVar2+m3*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4-m3*dqT*dqT*cos(qT+q1)*L4*cos(qT+q2)*ellzcm3+m3*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4*dq1+m3*dqT*dqT*L2*L2/2.0+m3*dqT*dqT*L4*L4/2.0+m3*L4*L4*dq1*dq1/2.0+mT*dqT*dqT*L2*L2/2.0+mT*L2*L2*dq2*dq2/2.0+mT*dqT*dqT*L4*L4/2.0+mT*L4*L4*dq1*dq1/2.0+m1*dqT*dqT*L2*L2/2.0+m2*L2*L2*dq2*dq2/2.0;
  MapleGenVar3 = MapleGenVar4+m2*L4*L4*dq1*dq1/2.0+m2*dqT*dqT*L4*L4/2.0+m2*dqT*dqT*L2*L2/2.0+m4*L4*L4*dq1*dq1/2.0+m4*dqT*dqT*L4*L4/2.0+m1*dqT*dqT*L4*L4/2.0+m1*dqT*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4-m2*dqT*dqT*sin(qT+q1)*L4*sin(qT+q2)*ellzcm2+m1*dqT*cos(qT+q1)*ellycm1*sin(qT+q2)*L2*dq2-m1*dqT*sin(qT+q1)*ellzcm1*sin(qT+q2)*L2*dq2+m1*dq1*cos(qT+q1)*ellycm1*sin(qT+q2)*L2*dq2-m1*dq1*sin(qT+q1)*ellzcm1*sin(qT+q2)*L2*dq2+m1*dqT*dqT*sin(qT+q2)*L2*cos(qT+q1)*ellycm1+m1*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4*dq1;
  MapleGenVar5 = MapleGenVar3+m1*sin(qT+q1)*L4*dq1*sin(qT+q2)*L2*dq2+m2*dqT*cos(qT+q1)*L4*cos(qT+q2)*L2*dq2-m2*dqT*dqT*cos(qT+q1)*L4*sin(qT+q2)*ellycm2-m2*dqT*sin(qT+q2)*ellzcm2*sin(qT+q1)*L4*dq1+m2*sin(qT+q1)*L4*dq1*dq2*cos(qT+q2)*ellycm2+m2*dqT*cos(qT+q2)*L2*cos(qT+q1)*L4*dq1;
  MapleGenVar4 = MapleGenVar5-m2*sin(qT+q1)*L4*dq1*dq2*sin(qT+q2)*ellzcm2-m2*dqT*cos(qT+q2)*ellzcm2*cos(qT+q1)*L4*dq1-m2*dqT*dqT*cos(qT+q1)*L4*cos(qT+q2)*ellzcm2+m3*dqT*dqT*sin(qT+q1)*L4*cos(qT+q2)*ellycm3+m3*dqT*sin(qT+q1)*L4*sin(qT+q2)*L2*dq2+m3*dqT*sin(qT+q2)*L2*sin(qT+q1)*L4*dq1+m3*sin(qT+q1)*L4*dq1*sin(qT+q2)*L2*dq2;
  MapleGenVar5 = MapleGenVar4-m3*dqT*sin(qT+q2)*L2*dq1*sin(qT+q1)*L1-m3*dqT*dqT*sin(qT+q1)*L4*sin(qT+q2)*ellzcm3-m3*dqT*dqT*cos(qT+q2)*L2*cos(qT+q1)*L1+m3*dqT*sin(qT+q1)*L4*dq2*cos(qT+q2)*ellycm3-m3*dqT*sin(qT+q1)*L4*dq2*sin(qT+q2)*ellzcm3-m3*dqT*sin(qT+q1)*L1*sin(qT+q2)*L2*dq2;
  double KE = MapleGenVar5+m3*dqT*cos(qT+q2)*ellycm3*sin(qT+q1)*L4*dq1-m3*dqT*sin(qT+q2)*ellzcm3*sin(qT+q1)*L4*dq1+m3*sin(qT+q1)*L4*dq1*dq2*cos(qT+q2)*ellycm3-m3*sin(qT+q1)*L4*dq1*dq2*sin(qT+q2)*ellzcm3-m3*dq1*sin(qT+q1)*L1*sin(qT+q2)*L2*dq2+m3*dqT*cos(qT+q1)*L4*cos(qT+q2)*L2*dq2+m3*cos(qT+q1)*L4*dq1*cos(qT+q2)*L2*dq2-2.0*m3*dqT*L4*dq1*L1;
  double Etot = PE + KE;
  double uLSpower, uLSspring, uLS;
  double y1 = SCP_SETPTLS - qLS;
  double dy1 = 0 - dqLS;
  // Controller choices
  switch(STANCE_CONTROLLER)
    {
    case 1:
      uLSpower = -75 * dqLS * (Etot - SCP_EDES); // Why -75?
      uLSspring = SCP_KPLS * y1 + SCP_KDLS * dy1;
      uLS = uLSpower + uLSspring;
      break;
    case 2:
      if(vcm < -0.5/2)
	{
	  uLSspring = (1.0/ALPHA) * SCP_KPLS * y1 + (1.0/ALPHA) * SCP_KDLS * dy1;
	  uLSpower = 0; // -25 * dqLS * (Etot - SCP_EDES);
	}
      else
	{
	  uLSpower = -50 * dqLS * (Etot - SCP_EDES); // Why -50?
	  uLSspring = ALPHA * SCP_KPLS * y1;
	}
      uLS = uLSpower + uLSspring;
      break;
    case 3:
      if(vcm < -0.5)
	{
	  uLS = (1.0/ALPHA) * SCP_KPLS * y1 + ALPHA * SCP_KDLS * dy1; // Not (1/ALPHA)?
	}
      else
	{
	  uLS =  ALPHA * (SCP_KPLS * y1 + SCP_KDLS * dy1);
	}
      break;
    default:
      using namespace std;
      cout << "Stance controller selected improperly:" << endl;
      cout << "Choices: 1, 2, or 3." << endl;
      cout << "Selected: " << STANCE_CONTROLLER << "." << endl;
      break;
    }
  double y2 = SCP_SETPTTDA - angles.torso;
  double dy2 = 0 - angles.dtorso;
  double uTorso = SCP_KPTDA * y2 + SCP_KDTDA * dy2;
  // Cap torques at +- 15
  output.torqueA = abs_max(-uTorso - (uLS / 2), MAX_TORQUE);
  output.torqueB = abs_max(-uTorso + (uLS / 2), MAX_TORQUE);
}

// Name: abs_max
// Description: Caps number to +- max
// Parameters:
//  number: Floating point number to be capped
//  max: Cap value
// Output: Result of capping
double abs_max(double number, double max)
{
  //  std::cout << "Capping: " << number << std::endl;
  if(abs(number) > max)
    {
      return (number > 0)? max : -max;
    }
  else
    {
      return number;
    }
}

/* Command line testing
   int main(int argc, char *argv[])
   {
   using namespace std;
   if(argc != 15)
   {
   cout << "14 Arguments are needed, only found: " << argc - 1 << endl;
   return 0;
   }
   else
   {
   AngleInputs angles;
   PositionInputs positions;
   angles.torso = atof(argv[1]);
   angles.motor1 = atof(argv[2]);
   angles.motor2 = atof(argv[3]);
   angles.gear1 = atof(argv[4]);
   angles.gear2 = atof(argv[5]);
   positions.Y = atof(argv[6]);
   positions.Z = atof(argv[7]);
   angles.dtorso = atof(argv[8]);
   angles.dmotor1 = atof(argv[9]);
   angles.dmotor2 = atof(argv[10]);
   angles.dgear1 = atof(argv[11]);
   angles.dgear2 = atof(argv[12]);
   positions.dY = atof(argv[13]);
   positions.dZ = atof(argv[14]);
   TorqueOutputs flight_output, stance_output;
   flight_state_controller(angles, positions, flight_output);
   cout << "Flight: Torque A: " << flight_output.torqueA << "; Torque B: " << flight_output.torqueB << endl;
   stance_state_controller(angles, stance_output);
   cout << "Stance: Torque A: " << stance_output.torqueA << "; Torque B: " << stance_output.torqueB << endl;
   return 1;
   }
   }*/
