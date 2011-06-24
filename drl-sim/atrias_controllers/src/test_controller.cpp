// Ported by: Colan Dray & Alireza Ramezani
// MATLAB code by: Jessy Grizzle

//#include <iostream>
//#include <stdlib.h>
//#include <math.h>
#include <atrias_controllers/controller.h>

#define PI 3.14159265
#define g 9.81
#define MAX_TORQUE 15.0

int counter = 0;
double FCP_SETPTLS = 0.7854;
double FCP_SETPTTDA = 0.0;
//double FCP_SETPTTDA = 0.1230;
const double FCP_KPLS = 400.0;
const double FCP_KDLS = 50.4077;
const double FCP_KPTDA = 750.0;
const double FCP_KDTDA = 50.0;
const double FCP_SETPTTDA_NOM = 0.0;
//const double FCP_SETPTTDA_NOM = 0.0524;
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
const double REF_SPEED = 1.3;
const int STANCE_CONTROLLER = 2;

struct TorqueOutputs
{
  double torqueA;
  double torqueB;
};

struct SensorInputs
{
  double torso;
  double motorA;
  double motorB;
  double legA;
  double legB;
  double dtorso;
  double dmotorA;
  double dmotorB;
  double dlegA;
  double dlegB;
  double Z;
  double dZ;
};

void flight_state_controller(SensorInputs sensors, TorqueOutputs& torques);
void stance_state_controller(SensorInputs sensors, TorqueOutputs& torques);
double abs_max(double var, double max);

extern void initialize_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
  TEST_CONTROLLER_STATE(state)->in_flight = true;
  output->motor_torqueA = output->motor_torqueB = 0.0;

  PRINT_MSG("Test Controller Initialized.\n");
}

extern void update_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
  SensorInputs sensors;
  TorqueOutputs torques;

  // Get sensor data
  // These are converted to Grizzle's reference frame
  sensors.torso = input->body_angle - PI / 2;
  sensors.motorA = input->motor_angleA - sensors.torso + PI / 2;
  sensors.motorB = input->motor_angleB - sensors.torso + PI / 2;
  sensors.legA = input->leg_angleA - sensors.torso + PI / 2;
  sensors.legB = input->leg_angleB - sensors.torso + PI / 2;
  sensors.dtorso = input->body_ang_vel;
  sensors.dmotorA = input->motor_velocityA;
  sensors.dmotorB = input->motor_velocityB;
  sensors.dlegA = input->leg_velocityA;
  sensors.dlegB = input->leg_velocityB;
  sensors.Z = input->height;
  sensors.dZ = input->vertical_velocity;

  double sprdefA = ABS(sensors.motorA - sensors.legA);
  double sprdefB = ABS(sensors.motorB - sensors.legB);
  double threshF = TEST_CONTROLLER_DATA(data)->flight_threshold;
  double threshS = TEST_CONTROLLER_DATA(data)->stance_threshold;
  double motgain = TEST_CONTROLLER_DATA(data)->motor_gain;

  //PRINT_MSG("\nA: %f\nB: %f\nF: %f\nS: %f", sprdefA, sprdefB, threshF, threshS);

  // Choose a controller based on state
  if(TEST_CONTROLLER_STATE(state)->in_flight)
    {
      flight_state_controller(sensors, torques);

      if (sprdefA > threshF || sprdefB > threshF)
	{
	  PRINT_MSG("Test controller status: LANDED.");
	  TEST_CONTROLLER_STATE(state)->in_flight = false;
	}
    }

  else
    {
      stance_state_controller(sensors, torques);

      if (sprdefA < threshS && sprdefB < threshS)
	{
	  PRINT_MSG("Test controller status: TAKEOFF.");
	  TEST_CONTROLLER_STATE(state)->in_flight = true;

	  double vcm = (mT*(cos(sensors.torso+sensors.motorB)*L2+cos(sensors.torso+sensors.motorA)*L4-sin(sensors.torso)*ellycmT-cos(sensors.torso)*ellzcmT)+m1*(cos(sensors.torso+sensors.motorB)*L2+cos(sensors.torso+sensors.motorA)*L4-sin(sensors.torso+sensors.motorA)*ellycm1-cos(sensors.torso+sensors.motorA)*ellzcm1)+m2*(cos(sensors.torso+sensors.motorB)*L2+cos(sensors.torso+sensors.motorA)*L4-sin(sensors.torso+sensors.motorB)*ellycm2-cos(sensors.torso+sensors.motorB)*ellzcm2)+m3*(cos(sensors.torso+sensors.motorB)*L2+cos(sensors.torso+sensors.motorA)*L4-cos(sensors.torso+sensors.motorA)*L1-sin(sensors.torso+sensors.motorB)*ellycm3-cos(sensors.torso+sensors.motorB)*ellzcm3)+m4*(cos(sensors.torso+sensors.motorA)*L4-sin(sensors.torso+sensors.motorA)*ellycm4-cos(sensors.torso+sensors.motorA)*ellzcm4))/(mT+m1+m2+m3+m4)*sensors.dtorso+(mT*cos(sensors.torso+sensors.motorA)*L4+m1*(cos(sensors.torso+sensors.motorA)*L4-sin(sensors.torso+sensors.motorA)*ellycm1-cos(sensors.torso+sensors.motorA)*ellzcm1)+m2*cos(sensors.torso+sensors.motorA)*L4+m3*(cos(sensors.torso+sensors.motorA)*L4-cos(sensors.torso+sensors.motorA)*L1)+m4*(cos(sensors.torso+sensors.motorA)*L4-sin(sensors.torso+sensors.motorA)*ellycm4-cos(sensors.torso+sensors.motorA)*ellzcm4))/(mT+m1+m2+m3+m4)*sensors.dmotorA+(mT*cos(sensors.torso+sensors.motorB)*L2+m1*cos(sensors.torso+sensors.motorB)*L2+m2*(cos(sensors.torso+sensors.motorB)*L2-sin(sensors.torso+sensors.motorB)*ellycm2-cos(sensors.torso+sensors.motorB)*ellzcm2)+m3*(cos(sensors.torso+sensors.motorB)*L2-sin(sensors.torso+sensors.motorB)*ellycm3-cos(sensors.torso+sensors.motorB)*ellzcm3))/(mT+m1+m2+m3+m4)*sensors.dmotorB;

	  double error_speed = REF_SPEED - vcm;
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
	      FCP_SETPTTDA = ((error_speed < 0)? -1 : 1) * FCP_SETPTTDA_NOM;
	    }
	}
    }

  // Send output torques to motors
  output->motor_torqueA = motgain * torques.torqueA;
  output->motor_torqueB = motgain * torques.torqueB;
}

extern void takedown_test_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
  output->motor_torqueA = output->motor_torqueB = 0.0;

  PRINT_MSG("Test Controller Stopped.\n");
}

// Name: flight_state_controller
// Description: Controller for flight state
// Parameters:
//  angles: Input angles
//  sensors: Input sensors
//  &torques: Pair of output torques
// Output: void
void flight_state_controller(SensorInputs sensors, TorqueOutputs& torques)
{
  // Math from file
  double vHip2 = sensors.dZ;
  double p42 = sensors.Z + cos(sensors.torso + sensors.motorB) * L2 + cos(sensors.torso + sensors.motorA) * L4;
  double qLA = (sensors.motorA + sensors.motorB) / 2;
  double dqLA = (sensors.dmotorA + sensors.dmotorB) / 2;
  double qLS = sensors.legB - sensors.legA;
  double dqLS = sensors.dlegB - sensors.dlegA;
  double qLA_abs = qLA + sensors.torso;
  double qTDA = qLA_abs - PI;
  double dqLA_abs = dqLA + sensors.dtorso;
  double dqTDA = dqLA_abs;
  double y2 = FCP_SETPTTDA - qTDA;
  double dy2 = 0 - dqTDA;
  double uLA = FCP_KPTDA * y2 + FCP_KDTDA * dy2;
  double y1, dy1, uLS;
  /* if( (vHip2 > 0) && (p42 < 0.05) )
    {
    FCP_SETPTLS += (10.0 * PI / 180);*/
      y1 = FCP_SETPTLS - qLS;
      dy1 = 0 - dqLS;
      uLS = FCP_KPLS * y1 + FCP_KDLS * dy1;/*
    }
  else
    {
      y1 = FCP_SETPTLS - qLS;
      dy1 = 0 - dqLS;
      uLS = FCP_KPLS * y1 + FCP_KDLS * dy1;
      }*/
  // Cap torques at +- 15
  torques.torqueA = 0 * abs_max(uLA - (uLS / 2), MAX_TORQUE);
  torques.torqueB = 0 * abs_max(uLA + (uLS / 2), MAX_TORQUE);
}

// Name: stance_state_controller
// Description: Controller for stance state
// Parameters:
//  sensors: Input sensors
//  &torques: Pair of output torques
// Output: void
void stance_state_controller(SensorInputs sensors, TorqueOutputs& torques)
{
  // Math from file
  double qLS = sensors.legB - sensors.legA;
  double dqLS = sensors.dlegB - sensors.dlegA;
  double vcm = (mT*(sin(sensors.torso+sensors.motorB)*L2+sin(sensors.torso+sensors.motorA)*L4+cos(sensors.torso)*ellycmT-sin(sensors.torso)*ellzcmT)+m1*(sin(sensors.torso+sensors.motorB)*L2+sin(sensors.torso+sensors.motorA)*L4+cos(sensors.torso+sensors.motorA)*ellycm1-sin(sensors.torso+sensors.motorA)*ellzcm1)+m2*(sin(sensors.torso+sensors.motorB)*L2+sin(sensors.torso+sensors.motorA)*L4+cos(sensors.torso+sensors.motorB)*ellycm2-sin(sensors.torso+sensors.motorB)*ellzcm2)+m3*(sin(sensors.torso+sensors.motorB)*L2+sin(sensors.torso+sensors.motorA)*L4-sin(sensors.torso+sensors.motorA)*L1+cos(sensors.torso+sensors.motorB)*ellycm3-sin(sensors.torso+sensors.motorB)*ellzcm3)+m4*(sin(sensors.torso+sensors.motorA)*L4+cos(sensors.torso+sensors.motorA)*ellycm4-sin(sensors.torso+sensors.motorA)*ellzcm4))/(mT+m1+m2+m3+m4)*sensors.dtorso+(mT*sin(sensors.torso+sensors.motorA)*L4+m1*(sin(sensors.torso+sensors.motorA)*L4+cos(sensors.torso+sensors.motorA)*ellycm1-sin(sensors.torso+sensors.motorA)*ellzcm1)+m2*sin(sensors.torso+sensors.motorA)*L4+m3*(sin(sensors.torso+sensors.motorA)*L4-sin(sensors.torso+sensors.motorA)*L1)+m4*(sin(sensors.torso+sensors.motorA)*L4+cos(sensors.torso+sensors.motorA)*ellycm4-sin(sensors.torso+sensors.motorA)*ellzcm4))/(mT+m1+m2+m3+m4)*sensors.dmotorA+(mT*sin(sensors.torso+sensors.motorB)*L2+m1*sin(sensors.torso+sensors.motorB)*L2+m2*(sin(sensors.torso+sensors.motorB)*L2+cos(sensors.torso+sensors.motorB)*ellycm2-sin(sensors.torso+sensors.motorB)*ellzcm2)+m3*(sin(sensors.torso+sensors.motorB)*L2+cos(sensors.torso+sensors.motorB)*ellycm3-sin(sensors.torso+sensors.motorB)*ellzcm3))/(mT+m1+m2+m3+m4)*sensors.dmotorB;
  double qT = sensors.torso;
  double q1 = sensors.motorA;
  double q2 = sensors.motorB;
  double qgr1 = sensors.legA;
  double qgr2 = sensors.legB;
  double dqT = sensors.dtorso;
  double dq1 = sensors.dmotorA;
  double dq2 = sensors.dmotorB;
  double dqgr1 = sensors.dlegA;
  double dqgr2 = sensors.dlegB;
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
      PRINT_MSG("Stance controller selected improperly:");
      break;
    }
  double y2 = SCP_SETPTTDA - sensors.torso;
  double dy2 = 0 - sensors.dtorso;
  double uTorso = SCP_KPTDA * y2 + SCP_KDTDA * dy2;
  // Cap torques at +- 15
  torques.torqueA = abs_max(-uTorso - (uLS / 2), MAX_TORQUE);
  torques.torqueB = abs_max(-uTorso + (uLS / 2), MAX_TORQUE);
}

// Name: abs_max
// Description: Caps number to +- max
// Parameters:
//  number: Floating point number to be capped
//  max: Cap value
// Output: Result of capping
double abs_max(double number, double max)
{
  if(ABS(number) > max)
    {
      return (number > 0)? max : -max;
      PRINT_MSG("CAPPED");
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
   AngleInputs sensors;
   PositionInputs sensors;
   sensors.torso = atof(argv[1]);
   sensors.motorA = atof(argv[2]);
   sensors.motorB = atof(argv[3]);
   sensors.legA = atof(argv[4]);
   sensors.legB = atof(argv[5]);
   sensors.Y = atof(argv[6]);
   sensors.Z = atof(argv[7]);
   sensors.dtorso = atof(argv[8]);
   sensors.dmotorA = atof(argv[9]);
   sensors.dmotorB = atof(argv[10]);
   sensors.dlegA = atof(argv[11]);
   sensors.dlegB = atof(argv[12]);
   sensors.dY = atof(argv[13]);
   sensors.dZ = atof(argv[14]);
   TorqueOutputs flight_output, stance_output;
   flight_state_controller(sensors, sensors, flight_output);
   cout << "Flight: Torque A: " << flight_output.torqueA << "; Torque B: " << flight_output.torqueB << endl;
   stance_state_controller(sensors, stance_output);
   cout << "Stance: Torque A: " << stance_output.torqueA << "; Torque B: " << stance_output.torqueB << endl;
   return 1;
   }
   }
*/
