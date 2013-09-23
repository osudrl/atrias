#include "atc_canonical_walking/ATCCanonicalWalking.hpp"

// The namespaces this controller resides in
namespace atrias {
  namespace controller {
   
    //Simple constructor 
    ATCCanonicalWalking::ATCCanonicalWalking(string name) :
      ATC(name),     
      pdLA(this, "pdLA"),
      pdLB(this, "pdLB"),
      pdLH(this, "pdLH"),
      pdRA(this, "pdRA"),
      pdRB(this, "pdRB"),
      pdRH(this, "pdRH"),
      rateLimLA(this, "rateLimLA"),
      rateLimLB(this, "rateLimLB"),
      rateLimLH(this, "rateLimLH"),
      rateLimRA(this, "rateLimRA"),
      rateLimRB(this, "rateLimRB"),
      rateLimRH(this, "rateLimRH")
    {
      // Do init here.
      // Startup is handled by the ATC class
      setStartupEnabled(true);

      // set the rate limit
      rate = 1.0;
      
      // initialize the walking parameter
      init_param();
      init_pos();
      
      // initial stance leg is right leg
      sleg = RIGHT_LEG;
      tau_prev = 0;
      cnt = 2;
      timer = 0;
    }
    
    void ATCCanonicalWalking::controller() {
      // The robot state is in an inherited member variable named rs
      // The controller output should be put in the inherited member co
      
      // Update gains, and other options
      updateController();
      
      // DRL Note: This has been disabled so as to allow each controller to
      // specify different hip controller behavior. Now the standing and walking
      // controllers call this themselves.
      //// Run the hip controller
      //hipController();
      
      // Main controller state machine
      switch (controllerState) 
	{
	case STANDING: // Stand upright to desired initial position
	  // Call Standing controller
	  standingController();
	  break;
	case WALKING:
	  // Call Walking controller
	  walkingController();

	  // @TODO: update stance leg 'sleg'
	  //        For initial tracking test, we will only test one step. 
	  //        Therefore, we don't need to update stance leg for 
	  //        initial test. This can be added later.
	  break;
	case STOPPING:
	  // Call Stopping controller
	  stoppingController();
	  break;
	}
    }
    
    void ATCCanonicalWalking::standingController(){
      isInitialized = true;

      /* The logOut class is an inherited member from ATC, and is of type
       * controller_log_data. guiIn, similarly, is of type
       * gui_to_controller
       */
      logOut.rqATgt = rateLimRA(pos_initial[0] - M_PI/2, rate);
      logOut.rqBTgt = rateLimRB(pos_initial[1] - M_PI/2, rate);
      logOut.lqATgt = rateLimLA(pos_initial[2] - M_PI/2, rate);
      logOut.lqBTgt = rateLimLB(pos_initial[3] - M_PI/2, rate);
      
      
      // Command the outputs (and copy to our logging data).
      // This is where the definition of ASCPD as a functor is convenient.
      // Legs
      // DRL Note: Added conversions of target position coordinates from Dr. Grizzle's system to ours.
      co.lLeg.motorCurrentA = pdLA(logOut.lqATgt, rs.lLeg.halfA.motorAngle, 0, rs.lLeg.halfA.motorVelocity);
      co.lLeg.motorCurrentB = pdLB(logOut.lqBTgt, rs.lLeg.halfB.motorAngle, 0, rs.lLeg.halfB.motorVelocity);
      co.rLeg.motorCurrentA = pdRA(logOut.rqATgt, rs.rLeg.halfA.motorAngle, 0, rs.rLeg.halfA.motorVelocity);
      co.rLeg.motorCurrentB = pdRB(logOut.rqBTgt, rs.rLeg.halfB.motorAngle, 0, rs.rLeg.halfB.motorVelocity);
      // @TODO: control hip motors here or not?
      // DRL Note: Inserting a call to the hip controller here, passing parameters that indicate that it
      // should smoothly initialize the hip position.
      hipController(true, false);

      // @TODO: add guiOut

    }
    
    void ATCCanonicalWalking::stoppingController(){
      // Compute and set motor currents (applies virtual dampers to all actuators)
      // @TODO: control hip motors here or not?
      co.lLeg.motorCurrentA = pdLA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
      co.lLeg.motorCurrentB = pdLB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
      co.rLeg.motorCurrentA = pdRA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
      co.rLeg.motorCurrentB = pdRB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);

      // DRL Note: Added a call to the hip controller function, telling it to do hip relaxation
      hipController(false, true);
    }

    void ATCCanonicalWalking::walkingController(){
      double tau, dtau, tau_d;

      // convert states
      convert2torso();

      // compute 'tau', 'dtau'
      tau = compute_tau();
      dtau = compute_dtau();
      // @TODO: time-based trajectory
      // tau = (rs.timing.controllerTime-initial_controllerTime) % PERIOD;
      // dtau = 1;

      if (isInitialized) 
	{
	  tau_prev = tau;
	  isInitialized = false;
	}

      // @hacks we might not need this.
      if(abs(tau - tau_prev) < 0.009)	// stuck
	timer++;
      else  				// walking	
	{
	  timer = 0; 
	  cnt = 2;
	}
      if ( timer >= 10 )
	{ 
	  cnt += 1;
	  timer  = 0; 
	}

      tau_d = tau + 0.007 * cnt;
      
      if(tau_d >  0.81)	   tau_d =  0.81;
      if(tau_d < -0.001)   tau_d = -0.001;

      // compute desired outputs
      compute_y2d(tau);
      compute_y2dDot(tau, dtau);
      
      // compute desired motor angles through inverse kinematics
      phi_inverse_mat();
      
      // convert states
      convert2bodypitch();

      /* The logOut class is an inherited member from ATC, and is of type
       * controller_log_data. guiIn, similarly, is of type
       * gui_to_controller
       */
      logOut.lqATgt = rateLimRA(qTgt[0], rate);
      logOut.lqBTgt = rateLimRB(qTgt[1], rate);
      logOut.rqATgt = rateLimLA(qTgt[2], rate);
      logOut.rqBTgt = rateLimLB(qTgt[3], rate);
      // @TODO: do we need to add rate limit on velocity?
      // DRL Note: We do not see any need for a rate limit on velocity.
      
      // Command the outputs (and copy to our logging data).
      // This is where the definition of ASCPD as a functor is convenient.
      // Legs
      co.lLeg.motorCurrentA = pdLA(logOut.lqATgt, rs.lLeg.halfA.motorAngle, dqTgt[0], rs.lLeg.halfA.motorVelocity);
      co.lLeg.motorCurrentB = pdLB(logOut.lqBTgt, rs.lLeg.halfB.motorAngle, dqTgt[1], rs.lLeg.halfB.motorVelocity);
      co.rLeg.motorCurrentA = pdRA(logOut.rqATgt, rs.rLeg.halfA.motorAngle, dqTgt[2], rs.rLeg.halfA.motorVelocity);
      co.rLeg.motorCurrentB = pdRB(logOut.rqBTgt, rs.rLeg.halfB.motorAngle, dqTgt[3], rs.rLeg.halfB.motorVelocity);
      // @TODO: control hip motors here or not?
      // DRL Note: Adding a call to hipController here.
      hipController(false, false);

      // @TODO: add guiOut

      
    }

	// @TODO: Maybe we don't need to include hip controller in this controller.
	// DRL Note: I've implemented all of this, including options used for the startup
	// and shutdown controllers.
	void ATCCanonicalWalking::hipController(bool start, bool stop) {
		// We handle the stopping state first, since it's the only one that doesn't
		// require doing the hip inverse kinematics and position control
		if (stop) {
			// Reset the hip rate limiters to the current hip positions
			rateLimLH.reset(rs.lleg.hip.legBodyAngle);
			rateLimRH.reset(rs.rleg.hip.legBodyAngle);

			// Do damping on the hip angles.
			co.lLeg.motorCurrentHip = pdLH(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
			co.rLeg.motorCurrentHip = pdRH(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);

			// Quit early -- we don't need to do position control.
		}

		// Set hip controller toe positions
		LeftRight toePosition;
		toePosition.left = guiIn.left_toe_pos;
		toePosition.right = guiIn.right_toe_pos;

		// Compute inverse kinematics to keep lateral knee torque to a minimum
		double lhTgt;
		double rhTgt;
		std::tie(lhTgt, rhTgt) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);

		// If we're not running the startup controller, set the hip position command directly
		// to the desired hip angles
		if (!start) {
			rateLimLH.reset(lhTgt);
			rateLimRH.reset(rhTgt);
		}

		// Do the rate limiting on hip position. The rate is currently hardcoded at .1 rad/s
		lhTgt = rateLimLH(lhTgt, .1);
		rhTgt = rateLimRH(rhTgt, .1);

		// Do position control on the hip positions
		co.lLeg.motorCurrentHip = pdLH(lhTgt, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
		co.rLeg.motorCurrentHip = pdRH(rhTgt, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
	}

    void ATCCanonicalWalking::updateController(){
      // the rate limiters if we're disabled. If not, disabling,
      // moving the robot or the setpoints, then enabling produces
      // a jump.
      if (!isEnabled()) {
	rateLimLA.reset(rs.lLeg.halfA.rotorAngle);
	rateLimLB.reset(rs.lLeg.halfB.rotorAngle);
	rateLimLH.reset(rs.lLeg.hip.legBodyAngle);
	rateLimRA.reset(rs.rLeg.halfA.rotorAngle);
	rateLimRB.reset(rs.rLeg.halfB.rotorAngle);
	rateLimRH.reset(rs.rLeg.hip.legBodyAngle);
      }
      
      // Set gains
      // Legs
      pdLA.P = pdLB.P = pdRA.P = pdRB.P = guiIn.leg_motor_p_gain;
      pdLA.D = pdLB.D = pdRA.D = pdRB.D = guiIn.leg_motor_d_gain;

      // DRL Note: Adding gain settings for hip PD controllers
      // Hips
      pdLH.P = PDRH.P = guiIn.hip_motor_p_gain;
      pdLH.D = PDRH.D = guiIn.hip_motor_d_gain;

      // Main controller options
      // DRL Note: Re-enabling this, so the user can control the controller state.
      controllerState = guiIn.main_controller;
    }

    
    
    /**
     * @brief This is to initialize the walking parameters.
     */
    void ATCCanonicalWalking::init_param(){
      // @TODO: We might need to figure out a proper way to initialize 
      //        parameter in the case it changes frequently.
      //        2 possible options: 1) From config file or 
      //                            2) From GUI input
      int i,j;
      for(i=0; i<N_OUTPUTS; i++)
	{
	  for(j=0; j<N_PARAMS; j++ )
	    {
	      param_mat[i][j] = a_opt[i][j];
	    }
	}

      theta_limit1	= 1.3602;
      theta_limit2	= 1.8282;
    }
    
    /**
     * @brief This is to set the initial position for walking.
     */
    void ATCCanonicalWalking::init_pos(){
      // @TODO: We might need to figure out a proper way to set 
      //        initial position in the case it changes frequently.
      //        2 possible options: 1) From config file or 
      //                            2) From GUI input

      // This is assuming starting from right leg stance.
      for(int i=0; i<N_MOTORS; i++)
	pos_initial[i] = x_opt[i];
      
    }
    
    /**
     * @brief This is to compute the inverse kinematics of the system.
     */
    void ATCCanonicalWalking::phi_inverse_mat(){
      int i,j;
      for(i=0; i<N_OUTPUTS; i++)
	{
	  for(j=0; j<N_OUTPUTS; j++ )
	    {
	      xd[i+1] += invT[i][j] * y2d[j];
	      xd[i+6] += invT[i][j] * y2dDot[j];
	    }
	}
      
    }
    
    /**
     * @brief This is to compute the phase (parameterized time) 'tau'
     */
    double ATCCanonicalWalking::compute_tau(){	
      //
      double th, tau;
      th = PI*3/2 - (xa[1]+(xa[2]+xa[3])/2);
      tau = (th-theta_limit1)/(theta_limit2-theta_limit1);
      return tau;
    }
    
    /**
     * @brief This is to compute the time derivative of 'tau'
     */
    double ATCCanonicalWalking::compute_dtau(){	
      //
      double dtau;
      dtau = -(xa[6]+(xa[7]+xa[8])/2)/(theta_limit2-theta_limit1);
      return dtau;
    }

    /**
     * @brief This is to compute the desired output 'y2d'
     * @param tau The parameterized time
     */
    void ATCCanonicalWalking::compute_y2d(double tau){
      // y2d = exp(-a4*t) * (a1*cos(a2*t)+a3*sin(a2*t)) + a5;
      //
      for(int j=0; j<N_OUTPUTS; j++)
	{ 
	  y2d[j] = exp(-param_mat[j][3]*tau) * 
	    (param_mat[j][0] * cos(param_mat[j][1]*tau) + 
	     param_mat[j][2] * sin(param_mat[j][1]*tau)) + 
	    param_mat[j][4];
	}
    }
    
    /**
     * @brief This is to compute the desired time derivative (velocity) of 'y2d'
     * @param tau The parameterized time
     * @param dtau The derivative of parameterized time
     */
    void ATCCanonicalWalking::compute_y2dDot(double tau, double dtau){
      // 
      for(int j=0; j<N_OUTPUTS; j++)
	{ 
	  y2dDot[j] =	exp(-param_mat[j][3]*tau) * 
	    ( param_mat[j][1] * param_mat[j][2]*cos(param_mat[j][1]*tau)- 
	      param_mat[j][0] * param_mat[j][1]*sin(param_mat[j][1]*tau))
	    - param_mat[j][3] * exp(-param_mat[j][3]*tau) *
	    ( param_mat[j][0] * cos(param_mat[j][1]*tau) +
	      param_mat[j][2] * sin(param_mat[j][1]*tau));
	  y2dDot[j] *= dtau;		//noted
	}
    }
    
    /**
     * @brief This is to convert the states from the current coordinate configuration to the old (Dr Grizzle's) configuration.
     */
    void  ATCCanonicalWalking::convert2torso(){
      //
      if (sleg == LEFT_LEG) 
	{
	  xa[0] = rs.position.bodyPitch - 3 * PI/2;
	  xa[1] = rs.lLeg.halfA.motorAngle + PI/2;
	  xa[2] = rs.lLeg.halfB.motorAngle + PI/2;
	  xa[3] = rs.rLeg.halfA.motorAngle + PI/2;
	  xa[4] = rs.rLeg.halfB.motorAngle + PI/2;
	  xa[5] = rs.position.bodyPitchVelocity;
	  xa[6] = rs.lLeg.halfA.motorVelocity;
	  xa[7] = rs.lLeg.halfB.motorVelocity;
	  xa[8] = rs.rLeg.halfA.motorVelocity;
	  xa[9] = rs.rLeg.halfB.motorVelocity;
	} 
      else 
	{ //sleg = RIGHT_LEG
	  xa[0] = rs.position.bodyPitch - 3 * PI/2;
	  xa[1] = rs.rLeg.halfA.motorAngle + PI/2;
	  xa[2] = rs.rLeg.halfB.motorAngle + PI/2;
	  xa[3] = rs.lLeg.halfA.motorAngle + PI/2;
	  xa[4] = rs.lLeg.halfB.motorAngle + PI/2;
	  xa[5] = rs.position.bodyPitchVelocity;
	  xa[6] = rs.rLeg.halfA.motorVelocity;
	  xa[7] = rs.rLeg.halfB.motorVelocity;
	  xa[8] = rs.lLeg.halfA.motorVelocity;
	  xa[9] = rs.lLeg.halfB.motorVelocity;
	}
      // 
    }
    
    
    /**
     * @brief This is to convert the states from the old (Dr. Grizzle's)  coordinate configuration to the current configuration.
     */
    void ATCCanonicalWalking::convert2bodypitch(){
      //
      if (sleg == LEFT_LEG) 
	{
	  qTgt[0] = xd[1] - PI/2;   // lLeg.halfA.motorAngle
	  qTgt[1] = xd[2] - PI/2;   // lLeg.halfB.motorAngle
	  qTgt[2] = xd[3] - PI/2;   // rLeg.halfA.motorAngle
	  qTgt[3] = xd[4] - PI/2;   // rLeg.halfB.motorAngle
	  dqTgt[0] = xd[6];         // lLeg.halfA.motorVelocity
	  dqTgt[1] = xd[7];         // lLeg.halfB.motorVelocity
	  dqTgt[2] = xd[8];         // rLeg.halfA.motorVelocity
	  dqTgt[3] = xd[9];         // rLeg.halfB.motorVelocity
	} 
      else 
	{ //sleg = RIGHT_LEG
	  qTgt[0] = xd[3] - PI/2;   // lLeg.halfA.motorAngle
	  qTgt[1] = xd[4] - PI/2;   // lLeg.halfB.motorAngle
	  qTgt[2] = xd[1] - PI/2;   // rLeg.halfA.motorAngle
	  qTgt[3] = xd[2] - PI/2;   // rLeg.halfB.motorAngle
	  dqTgt[0] = xd[8];         // lLeg.halfA.motorVelocity
	  dqTgt[1] = xd[9];         // lLeg.halfB.motorVelocity
	  dqTgt[2] = xd[6];         // rLeg.halfA.motorVelocity
	  dqTgt[3] = xd[7];         // rLeg.halfB.motorVelocity
	}
      // 
    }
    
    
    ORO_CREATE_COMPONENT(ATCCanonicalWalking)
    
  }
}


