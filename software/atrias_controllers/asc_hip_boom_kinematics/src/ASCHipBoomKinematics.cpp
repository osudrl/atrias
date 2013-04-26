#include "asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ASCHipBoomKinematics::ASCHipBoomKinematics(AtriasController *parent, string name) :
        AtriasController(parent, name),
        log_out(this, "log")
{
	// Distance from boom pivot Z axis to robot body XZ center plane along boom Y axis
    lBoom = 2.04;
    
    // Distance from boom Y axis intersection with robot body XZ center plane to hip pivot X axis
    lBody = 0.35;
    
    // Distance from hip pivot X axis to XZ center plane of leg assembly
    lHip = 0.18;
    
    // Angle between boom Y axis and robot body XZ center plane
    qBodyOffset = PI/2.0 - 0.126;
}


std::tuple<double, double> ASCHipBoomKinematics::iKine(LeftRight toePosition, atrias_msgs::robot_state_leg lLeg, atrias_msgs::robot_state_leg rLeg, atrias_msgs::robot_state_location position) {

    // Define imaginary number i
    i = complex<double>(0.0, 1.0);

	// Get leg lengths and angles
    lLeftLeg = (L1 + L2)*cos((lLeg.halfB.legAngle - lLeg.halfA.legAngle)/2.0);
    lRightLeg = (L1 + L2)*cos((rLeg.halfB.legAngle - rLeg.halfA.legAngle)/2.0);
    qLeftLeg = (lLeg.halfA.legAngle + lLeg.halfB.legAngle)/2.0;
    qRightLeg = (rLeg.halfA.legAngle + rLeg.halfB.legAngle)/2.0;

	// Compute inverse kinematics
	complexHipAngleLeft = - position.boomAngle - qBodyOffset - log((- sqrt(pow(lLeftLeg, 2) - 2.0*pow(lLeftLeg, 2)*exp(qLeftLeg*2.0*i) + pow(lLeftLeg, 2)*exp(qLeftLeg*4.0*i) + 4.0*pow(toePosition.left, 2)*exp(qLeftLeg*2.0*i) - 4.0*pow(lHip, 2)*exp(qLeftLeg*2.0*i) + 4.0*pow(lBoom, 2)*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2) - 4.0*pow(lLeftLeg, 2)*exp(qLeftLeg*2.0*i)*pow(cos(qLeftLeg), 2) + 4.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2)*pow(cos(qBodyOffset), 2) + 4.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*pow(sin(position.boomAngle), 2)*pow(sin(qBodyOffset), 2) + 8.0*lBoom*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) + 8.0*lBoom*lBody*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2)*cos(qBodyOffset) - 8.0*lBoom*lBody*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*sin(position.boomAngle)*sin(qBodyOffset) + 8.0*lBody*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*cos(qBodyOffset)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) - 8.0*lBody*exp(qLeftLeg*2.0*i)*sin(position.boomAngle)*sin(qBodyOffset)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) - 8.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*cos(qBodyOffset)*sin(position.boomAngle)*sin(qBodyOffset)) + 2.0*lBoom*cos(position.boomAngle)*(cos(qLeftLeg) + sin(qLeftLeg)*i) + 2.0*exp(qLeftLeg*i)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) + 2.0*lBody*cos(position.boomAngle + qBodyOffset)*(cos(qLeftLeg) + sin(qLeftLeg)*i))/(lLeftLeg + 2.0*lHip*exp(qLeftLeg*i) - lLeftLeg*exp(qLeftLeg*2.0*i)))*i;

    complexHipAngleRight = - position.boomAngle - qBodyOffset - log(-(- sqrt(2.0*pow(lBody, 2)*exp(qRightLeg*2.0*i) - 4.0*pow(lHip, 2)*exp(qRightLeg*2.0*i) - 2.0*pow(lRightLeg, 2)*exp(qRightLeg*2.0*i) + pow(lRightLeg, 2)*exp(qRightLeg*4.0*i) + 4.0*pow(toePosition.right, 2)*exp(qRightLeg*2.0*i) + pow(lRightLeg, 2) + 4.0*pow(lBoom, 2)*exp(qRightLeg*2.0*i)*pow(cos(position.boomAngle), 2) - 4.0*pow(lRightLeg, 2)*exp(qRightLeg*2.0*i)*pow(cos(qRightLeg), 2) + 2.0*pow(lBody, 2)*cos(2.0*position.boomAngle + 2.0*qBodyOffset)*exp(qRightLeg*2.0*i) + 4.0*lBoom*lBody*exp(qRightLeg*2.0*i)*cos(qBodyOffset) + 4.0*lBoom*lBody*cos(2.0*position.boomAngle + qBodyOffset)*exp(qRightLeg*2.0*i) + 8.0*lBody*exp(qRightLeg*2.0*i)*cos(position.boomAngle + qBodyOffset)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg)) + 8.0*lBoom*exp(qRightLeg*2.0*i)*cos(position.boomAngle)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg))) + 2.0*exp(qRightLeg*i)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg)) + 2.0*lBody*exp(qRightLeg*i)*cos(position.boomAngle + qBodyOffset) + 2.0*lBoom*exp(qRightLeg*i)*cos(position.boomAngle))/(- lRightLeg + 2.0*lHip*exp(qRightLeg*i) + lRightLeg*exp(qRightLeg*2.0*i)))*i;

	// TODO - Add velocity terms

	// We only care about the real part, the imaginary part should be zero
	hipAngle.left = fmod(real(complexHipAngleLeft) + 4.0*PI, 2.0*PI);
	hipAngle.right = fmod(real(complexHipAngleRight) + 4.0*PI, 2.0*PI);

	// TODO - Clamp hip angles to physical limits
	
	// Set the log data
    log_out.data.leftHipAngle = hipAngle.left;
    log_out.data.rightHipAngle = hipAngle.right;

    // Transmit the log data
    log_out.send();

	// Return the computed hip angles	
	return std::make_tuple(hipAngle.left, hipAngle.right);

}

}
}
