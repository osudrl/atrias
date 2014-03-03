#ifndef ATCCanonicalWalking_HPP
#define ATCCanonicalWalking_HPP

/**
 * @file ATCCanonicalWalking.hpp
 * @author Ryan Van Why
 * @brief This controller is to test the tracking of canonical walking
 */


// Top-level controllers are components, so we need to include this.
#include <rtt/Component.hpp>

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>

// Our logging data type.
#include "atc_canonical_walking/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "atc_canonical_walking/controller_input.h"
// The type transmitted from the controller to the GUI
#include "atc_canonical_walking/controller_status.h"


// Our subcontroller types
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
#include <asc_pd/ASCPD.hpp>
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>
#define N_DOFS 5
#define N_OUTPUTS 4
#define N_MOTORS 4
#define N_STATES 10
#define N_PARAMS 7

// This controller's common definitions
#include "common.hpp"


// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
  namespace controller {
    
    static const double invT[N_OUTPUTS][N_MOTORS] = {
      {1  ,   0,   0,   0},
      {0  ,   1,   0,   0},
      {0  ,   0,   1,   0},
      {0  ,   0,   0,   1}};

  static const double A_OPT[N_OUTPUTS][N_PARAMS] = {
    {3.07264014471119,	3.20203467410224,	2.45069759936780,	3.51164791250083,	2.31473515689294,	2.87712976645530,	2.59904522713929},
    {3.82025885399207,	3.72604682489029,	3.63600166378923,	3.59064706271301,	3.43330071754072,	3.43555758558403,	3.33091321903808},
    {2.59905548740630,	2.27875192891540,	3.23617561497869,	1.08168358636043,	2.85096809722546,	2.96061167285344,	3.07295456487627},
    {3.33090847276033,	3.21037963591491,	2.83194701949237,	3.76315528185840,	3.97034021548090,	3.90194131813320,	3.82013779297009}};




  static const double D_OPT[N_OUTPUTS][N_PARAMS-1] = {
    {0.776367176346266,	-4.50802244840664,	6.36570187879821,	-7.18147653364737,	3.37436765737415,	-1.66850723589601},
    {-0.565272174610634,	-0.540270966606393,-0.272127606457296,	-0.944078071033765,	0.0135412082598627,	-0.627866199275704},
    {-1.92182135094540,	5.74454211637972,	-12.9269521717095,	10.6157070651902,	0.657861453767900,	0.674057352136984},
    {-0.723173021072473,	-2.27059569853527,	5.58724957419622,	1.24310960173495,	-0.410393384086174,	-0.490821150978650}};

 
    /*static const double x_opt[4] = {3.08386532926800,   //rLeg.halfA.motorAngle
      3.07449603435955,   //rLeg.halfB.motorAngle
      3.43731571117661,   //lLeg.halfA.motorAngle
      3.69996901140680};    //lLeg.halfB.motorAngle*/
    static const double X_OPT[4] =  {2.59905548740630,
				     3.33090847276033,
				     3.07264014471119,
				     3.82025885399207};
  



  static const double P_LIMITS[2] ={-0.286373623667230,	0.177718855319307};



    
    class ATCCanonicalWalking : public ATC<
      atc_canonical_walking::controller_log_data_,
      atc_canonical_walking::controller_input_,
      atc_canonical_walking::controller_status_>
    {
    public:
      /** 
       * @brief The constructor for this controller.
       * @param name The name of this component.
       * Every top-level controller will have this name parameter,
       * just like current controllers.
       */
      ATCCanonicalWalking(string name);
      
    private:
      /** 
       * @brief This is the main function for the top-level controller.
       */
      void controller();
      
      /** 
       * @brief Those are top level controllers.
       */ 
      void updateController();
      // DRL Note: I've added parameters to allow for smooth initialization and shutdown of the hips
      void hipController(bool start, bool stop);
      void standingController();
      void stoppingController();
      void walkingController();

      /**
        * @brief This function places a torque limit on the output of the controller.
        */
      void clampTorques();

      /**
        * @brief This function checks if an EStop needs to occur (based on the limits), and triggers
        * the estop if necessary
        */
      void checkLimits();

      // Include subcontrollers and variables here
      // Hip inverse kinematics subcontroller
      ASCHipBoomKinematics ascHipBoomKinematics;
      
      

      // PD controllers for each motor
      ASCPD pdLA; // Left  A
      ASCPD pdLB; // Left  B
      ASCPD pdLH; // Left  H
      ASCPD pdRA; // Right A
      ASCPD pdRB; // Right B
      ASCPD pdRH; // Right H

      // Rate limiters for each motor
      ASCRateLimit rateLimLA; // Left  A
      ASCRateLimit rateLimLB; // Left  B
      ASCRateLimit rateLimLH; // Left Hip
      ASCRateLimit rateLimRA; // Right A
      ASCRateLimit rateLimRB; // Right B
      ASCRateLimit rateLimRH; // Right Hip
      // And one for tau
      ASCRateLimit rateLimTau;
      
      /**
       * @define Transformation Matrix and Its Inverse Matrix
       *  y2 = T * states;
       *  states = invT * y2;
       *
       * double T[4][4] = {
       *       {1/2, 1/2,   0,   0},
       *       {0  ,   0, 1/2, 1/2},
       *       {-1 ,   1,   0,   0},
       *       {0  ,   0,  -1,   1}};
       */
      //double invT[N_OUTPUTS][N_OUTPUTS];
      //static const double invT, a_opt, x_opt;

      // define varibles
      double theta_limit1;  // Initial phase (or parameterized time)
      double theta_limit2;  // Final phase (or parameterized time)
      
      double vhip;          // desired hip velocity
      double phip0;         // initial hip position

      double param_mat[N_OUTPUTS][N_PARAMS];  // The parameter matrix of Canonical Walking Function.
      double diff_param_mat[N_OUTPUTS][N_PARAMS-1];
      /**
       * @define y2d (y2dDot) Outputs: sLegAngle, NsLegAngle, sKnee, NsKnee
       */
      double y2d[N_OUTPUTS];    // The desired outputs
      double y2dDot[N_OUTPUTS]; // The desired time derivative of outputs

      /**
       * @define robot states: {qxT, q1, q2, q1L, q2L, dqxT, dq1, dq2, dq1L, dq2L}
       * **       q1, q2 are the stance leg motor anlges;
       * **       q1L, q2L are the non-stance leg motor angles;
       * **       qxT is torso angle.
       */
      double xa[N_STATES];      // The current robot states in the old configuration.  
      double xd[N_STATES];      // The desired robot states in the old configuration.
      
      double pos_initial[N_MOTORS]; // The initial position for walking
      
      /**
       * @define desired motor angles: {qlATgt, qlBTgt, qrATgt, qrBTgt}
       */
      double qTgt[N_MOTORS];    // The desired motor position in the current configuration.
      double dqTgt[N_MOTORS];   // The desired motor velocity in the current configuration.
     
      enum StanceLeg {LEFT_LEG, RIGHT_LEG};
      StanceLeg sleg, stance_prev;
      enum CtrlState {STANDING = 0, WALKING = 1, STOPPING = 2};
      CtrlState controllerState;

      // The maximum rate of motion
      double rate;
      
      bool    isInitialized;
      double tau_prev;
      int    cnt;
      int    timer;
      


      // Define subfunctions
      /**
       * @brief This is to initialize the walking parameters.
       */
      void init_param();
      
      /**
       * @brief This is to set the initial position for walking.
       */
      void init_pos();
      
      /**
       * @brief This is to compute the inverse kinematics of the system.
       */
      void phi_inverse_mat();

      /**
       * @brief This is to compute the phase (parameterized time) 'tau'
       */
      double compute_tau();
      
      /**
       * @brief This is to compute the time derivative of 'tau'
       */
      double compute_dtau();

      /**
       * @brief This is to compute the desired output 'y2d'
       * @param tau The parameterized time
       */
      void compute_y2d(double tau);

      /**
       * @brief This is to compute the desired time derivative (velocity) of 'y2d'
       * @param tau The parameterized time
       * @param dtau The derivative of parameterized time
       */
      void compute_y2dDot(double tau, double dtau);

      /**
       * @brief This is to convert the states from the current coordinate configuration to the old (Dr Grizzle's) configuration.
       */
      void convert2torso();

      /**
       * @brief This is to convert the states from the old (Dr. Grizzle's)  coordinate configuration to the current configuration.
       */
      void convert2bodypitch();

      

    };
    
  }
}

#endif // ATCCanonicalWalking_HPP
