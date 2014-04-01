#include "atc_stabilized_standing/ATCStabilizedStanding.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ATCStabilizedStanding::ATCStabilizedStanding(string name) :
  ATC(name),
  ascCommonToolkit(this, "ascCommonToolkit"),
  ascHipBoomKinematics(this, "ascHipBoomKinematics"),
  ascPDLmA(this, "ascPDLmA"),
  ascPDLmB(this, "ascPDLmB"),
  ascPDRmA(this, "ascPDRmA"),
  ascPDRmB(this, "ascPDRmB"),
  ascPDLh(this, "ascPDLh"),
  ascPDRh(this, "ascPDRh"),
  ascRateLimitLh(this, "ascRateLimitLh"),
  ascRateLimitLmA(this, "ascRateLimitLmA"),
  ascRateLimitLmB(this, "ascRateLimitLmB"),
  ascRateLimitRh(this, "ascRateLimitRh"),
  ascRateLimitRmA(this, "ascRateLimitRmA"),
  ascRateLimitRmB(this, "ascRateLimitRmB")
{
  // Startup is handled by the ATC class
  setStartupEnabled(true);

  // Set hard coded limits
  legMotorRateLimit = 0.5; // [rad/s]
  hipMotorRateLimit = 0.5; // [rad/s]
  currentLimit = 30.0; // [amps]
  deflectionLimit = 0.3; // [rad]
  velocityLimit = 6.0; // [rad/s]
}

/**
 * @brief Top-level controller.
 * 
 * This is the main function for the top-level controller.
 * The ATC class automatically handles startup and shutdown,
 * if they are not disabled.
 */
void ATCStabilizedStanding::controller() {
  // Update GUI values, gains, and other options
  updateController();

  // Run the hip controller
  hipController();

  // Main controller state machine
  switch (controllerState) {
    case 0: // Startup
      // Call startup controller
      startupController();
      break;

    case 1: // Stabilized standing
      // Stablilized single leg standing controller
      stabilizationController();
      break;

    case 2: // Shutdown
      // Call shutdown controller
      shutdownController();
      break;
  } // switch

  // Run safety checks and trigger E-stop if needed
  checkSafeties();
} // controller

/**
 * @brief Update controller parameters.
 * 
 * This function handles all of the non-controller related
 *  updating and all communication to and from the GUI.
 */
void ATCStabilizedStanding::updateController() {
  // If we are disabled or the controller has been switched, reset rate limiters
  if (!(isEnabled()) || !(controllerState == guiIn.main_controller)) {
    ascRateLimitLh.reset(rs.lLeg.hip.legBodyAngle);
    ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
    ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
    ascRateLimitRh.reset(rs.rLeg.hip.legBodyAngle);
    ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
    ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
  }

  // Main controller options
  controllerState = guiIn.main_controller;

  // Return is enabled
  guiOut.isEnabled = isEnabled();
} // updateController

/**
 * @brief Run safety checks.
 * 
 * This function performs various safety checks and triggers a software
 * emergency stop if needed.
 */
void ATCStabilizedStanding::checkSafeties() {
  // Limit motor currents to GUI specified value
  co.lLeg.motorCurrentA = clamp(co.lLeg.motorCurrentA, -currentLimit, currentLimit);
  co.lLeg.motorCurrentB = clamp(co.lLeg.motorCurrentB, -currentLimit, currentLimit);
  co.rLeg.motorCurrentA = clamp(co.rLeg.motorCurrentA, -currentLimit, currentLimit);
  co.rLeg.motorCurrentB = clamp(co.rLeg.motorCurrentB, -currentLimit, currentLimit);

  // Trigger an E-stop if a motor velocity goes over the hard coded limit
  if (abs(rs.lLeg.halfA.rotorVelocity) > velocityLimit ||
    abs(rs.lLeg.halfB.rotorVelocity) > velocityLimit ||
    abs(rs.rLeg.halfA.rotorVelocity) > velocityLimit ||
    abs(rs.rLeg.halfB.rotorVelocity) > velocityLimit ||
    abs(rs.lLeg.hip.legBodyVelocity) > velocityLimit ||
    abs(rs.rLeg.hip.legBodyVelocity) > velocityLimit)
  {
    // Trigger E-stop
    printf("Software E-Stop triggered by motor velocity limit check.\n");
    printf("GUI velocityLimit: %f\n", velocityLimit);
    commandEStop();
  } // if

  // Trigger an E-stop if a spring deflection goes over the hard coded limit
  if (abs(rs.lLeg.halfA.motorAngle - rs.lLeg.halfA.legAngle) > deflectionLimit ||
    abs(rs.lLeg.halfB.motorAngle - rs.lLeg.halfB.legAngle) > deflectionLimit ||
    abs(rs.rLeg.halfA.motorAngle - rs.rLeg.halfA.legAngle) > deflectionLimit ||
    abs(rs.rLeg.halfB.motorAngle - rs.rLeg.halfB.legAngle) > deflectionLimit)
  {
    // Trigger E-stop
    printf("Software E-Stop triggered by spring deflection limit check.\n");
    printf("GUI deflectionLimit: %f\n", deflectionLimit);
    commandEStop();
  } // if
} // checkSafeties

/**
 * @brief Hip position tracking controller.
 * 
 * This function handles all hip motor commands independent of other
 * functions. It works by computing the inverse kinematics of the robot
 * selecting hip angles that result in the desired toe positions. This keeps
 * knee torques to a minimum.
 */
void ATCStabilizedStanding::hipController() {
  // Set hip controller toe positions
  toePosition.left = 2.17;
  toePosition.right = 2.5;

  // Compute inverse kinematics to keep lateral knee torque to a minimum
  std::tie(qLh, qRh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);

  // Rate limit motor velocities to smooth step inputs
  qLh = ascRateLimitLh(qLh, hipMotorRateLimit);
  qRh = ascRateLimitRh(qRh, hipMotorRateLimit);

  // Compute and set motor currents from position based PD controllers
  co.lLeg.motorCurrentHip = ascPDLh(qLh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
  co.rLeg.motorCurrentHip = ascPDRh(qRh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
} // hipController

/**
 * @brief Startup controller.
 * 
 * This function uses position control on the leg motors to allow the
 * robot to stand with the torso locked.
 */
void ATCStabilizedStanding::startupController() {
  // Compute target motor angles
  std::tie(qmLA, qmLB) = ascCommonToolkit.legPos2MotorPos(M_PI/2.0, 0.9);
  std::tie(qmRA, qmRB) = ascCommonToolkit.legPos2MotorPos(M_PI/2.0, 0.9);

  // Rate limit motor velocities
  qmLA = ascRateLimitLmA(qmLA, legMotorRateLimit);
  qmLB = ascRateLimitLmB(qmLB, legMotorRateLimit);
  qmRA = ascRateLimitRmA(qmRA, legMotorRateLimit);
  qmRB = ascRateLimitRmB(qmRB, legMotorRateLimit);

  // Compute and set motor currents
  co.lLeg.motorCurrentA = ascPDLmA(qmLA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
  co.lLeg.motorCurrentB = ascPDLmB(qmLB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
  co.rLeg.motorCurrentA = ascPDRmA(qmRA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
  co.rLeg.motorCurrentB = ascPDRmB(qmRB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
} // startupController

/**
 * @brief Soft robot shutdown controller.
 * 
 * This function uses a Linear Quadratic Regulator to stabilize the 
 * robot while standing on a single leg.
 */
void ATCStabilizedStanding::stabilizationController() {
  // Reset control inputs U = [tauLmA, tauLmB, tauRmA, tauRmB]
  for (i=0; i<4; ++i) {
    U[i] = UStar[i];
  } // for(i)

  // Reset states X = [qt, qLmA, qLmB, qRlA, qRlB, qRmA, qRmB]
  X[0] = 2.0*M_PI - rs.position.bodyPitch;
  X[1] = - rs.position.bodyPitchVelocity;
  X[2] = 3.0*M_PI/2.0 - rs.lLeg.halfA.rotorAngle;
  X[3] = - rs.lLeg.halfA.rotorVelocity;
  X[4] = 3.0*M_PI/2.0 - rs.lLeg.halfB.rotorAngle;
  X[5] = - rs.lLeg.halfB.rotorVelocity;
  X[6] = 3.0*M_PI/2.0 - rs.rLeg.halfA.legAngle;
  X[7] = - rs.rLeg.halfA.legVelocity;
  X[8] = 3.0*M_PI/2.0 - rs.rLeg.halfB.legAngle;
  X[9] = - rs.rLeg.halfB.legVelocity;
  X[10] = 3.0*M_PI/2.0 - rs.rLeg.halfA.rotorAngle;
  X[11] = - rs.rLeg.halfA.rotorVelocity;
  X[12] = 3.0*M_PI/2.0 - rs.rLeg.halfB.rotorAngle;
  X[13] = - rs.rLeg.halfB.rotorVelocity;

  // Compute control inputs
  for(i=0; i<4; i++) {
    for(j=0; j<14; j++) {
      U[i] += K[i][j] * (XStar[j] - X[j]);
    } // for(j)
  } // for(i)

  // Set motor currents [tauLmA, tauLmB, tauRmA, tauRmB]
  co.lLeg.motorCurrentA = U[1]/50/0.0987;
  co.lLeg.motorCurrentB = U[2]/50/0.0987;
  co.rLeg.motorCurrentA = U[3]/50/0.0987;
  co.rLeg.motorCurrentB = U[4]/50/0.0987;
} // stabilizationController

/**
 * @brief Soft robot shutdown controller.
 * 
 * This function imposes virtual dampers on each motor allowing the
 * robot to safely and slowly shutdown.
 */
void ATCStabilizedStanding::shutdownController() {
  // Compute and set motor currents (applies virtual dampers to all actuators)
  co.lLeg.motorCurrentA = ascPDLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
  co.lLeg.motorCurrentB = ascPDLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
  co.lLeg.motorCurrentHip = ascPDLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
  co.rLeg.motorCurrentA = ascPDRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
  co.rLeg.motorCurrentB = ascPDRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
  co.rLeg.motorCurrentHip = ascPDRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);
} // shutdownController

ORO_CREATE_COMPONENT(ATCStabilizedStanding)

}
}
