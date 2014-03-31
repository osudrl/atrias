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
    {3.13058278616339,	3.38621285654106,	2.26312258300192,	3.72562917760651,	2.21768467271887,	3.00343594477559,	2.63518033870867},
    {3.88033369474228,	3.81628375091666,	3.64791062079225,	3.54302625758800,	3.60125936599999	3.45364342671221	3.37022239691970},
    {2.63529972490620,	2.22706769173603,	3.56813326557356,	0.695310365268170,	2.91237158193484,	2.90041027511901,	3.13100976580989},
    {3.37031174388957,	3.27783500873126,	2.93958409614047,	3.48644831132767,	4.39294714621698,	3.93787692520441,	3.88009422366450}};




  static const double D_OPT[N_OUTPUTS][N_PARAMS-1] = {
    { 1.53378042226602,	-6.73854164123482,	8.77503956762755,	-9.04766702932585,	4.71450763234033,	-2.20953363640152},
    {-0.384299662953726,	-1.01023878074645,	-0.629306179225527,	0.349398650471944,	-0.885695635726687,	-0.500526178755031},
    {-2.44939219902102,	8.04639344302521,	-17.2369374018324,	13.3023673000000,	-0.0717678408949780,	1.38359694414531},
    {-0.554860410949804 ,-2.02950547554476,	3.28118529112322,	5.43899300933585,	-2.73042132607544,	-0.346696209239457}};

 
    /*static const double x_opt[4] = {3.08386532926800,   //rLeg.halfA.motorAngle
      3.07449603435955,   //rLeg.halfB.motorAngle
      3.43731571117661,   //lLeg.halfA.motorAngle
      3.69996901140680};    //lLeg.halfB.motorAngle*/
    static const double X_OPT[4] =  {2.63529972490620,
				     3.37031174388957,
				     3.13058278616339,
				     3.88033369474228};
  



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
