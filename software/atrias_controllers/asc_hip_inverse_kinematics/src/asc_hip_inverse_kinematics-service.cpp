// Title: ASCHipInverseKinematics
// Description: Defines hip angle and toe position relationships.
// Author: Mikhail Jones

// Usage: To use this subcontroller, add the following pieces of code to the the designated files in your controller folder.

// Manifest.xml
// - <depend package="asc_hip_inverse_kinematics"/>

// Start.ops
// - # ASCHipInverseKinematics
// - require("ascHipInverseKinematics")
// - loadService("controller", "ascHipInverseKinematics")

// component_controller.cpp
// - // ASCHipInverseKinematics Service
// - toePositionToHipAngle = this->provides("ascHipInverseKinematics")->getOperation("toePositionToHipAngle");

// component_controller.h
// - // ASCHipInverseKinematics
// - OperationCaller<HipAngle(ToePosition toePosition, Leg lLeg, Leg rLeg, Position position)> toePositionToHipAngle;

// To use do something like this.
// - hipAngle = toePositionToHipAngle(toePosition, lLeg, rLeg, position);

// TODO - Use predefined robot state structure.

#include <asc_hip_inverse_kinematics/asc_hip_inverse_kinematics-service.h>

namespace atrias {
namespace controller {

// ASCHipInverseKinematics Constructor =========================================
ASCHipInverseKinematics::ASCHipInverseKinematics(TaskContext* owner)
    : Service("ascHipInverseKinematics", owner) {
    this->addOperation("toePositionToHipAngle",&ASCHipInverseKinematics::toePositionToHipAngle, this).doc("Given a boom angle, leg angles, and desired toe positions, returns required hip angles");
}


// ASCHipInverseKinematics::toePositionToHipAngle ==============================
HipAngle ASCHipInverseKinematics::toePositionToHipAngle(ToePosition toePosition, Leg lLeg, Leg rLeg, Position position) {

    // Define imaginary number i.
    i = complex<double>(0.0, 1.0);

	// Define robot parameters.
	l1 = 0.50;
	l2 = 0.50;
    lBoom = 2.04; // Distance from boom pivot Z axis to robot body XZ center plane along boom Y axis.
    lBody = 0.35; // Distance from boom Y axis intersection with robot body XZ center plane to hip pivot X axis.
    lHip = 0.18; // Distance from hip pivot X axis to XZ center plane of leg assembly.
    qBodyOffset = M_PI/2.0 - 0.126; // Angle between boom Y axis and robot body XZ center plane.

	// TODO - Get leg lengths and angles from robot state?
    lLeftLeg = (l1 + l2)*cos((lLeg.halfB.legAngle - lLeg.halfA.legAngle)/2.0);
    lRightLeg = (l1 + l2)*cos((rLeg.halfB.legAngle - rLeg.halfA.legAngle)/2.0);
    qLeftLeg = (lLeg.halfA.legAngle + lLeg.halfB.legAngle)/2.0;
    qRightLeg = (rLeg.halfA.legAngle + rLeg.halfB.legAngle)/2.0;

	// Compute inverse kinematics
	complexHipAngleLeft = - position.boomAngle - qBodyOffset - log((- sqrt(pow(lLeftLeg, 2) - 2.0*pow(lLeftLeg, 2)*exp(qLeftLeg*2.0*i) + pow(lLeftLeg, 2)*exp(qLeftLeg*4.0*i) + 4.0*pow(toePosition.left, 2)*exp(qLeftLeg*2.0*i) - 4.0*pow(lHip, 2)*exp(qLeftLeg*2.0*i) + 4.0*pow(lBoom, 2)*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2) - 4.0*pow(lLeftLeg, 2)*exp(qLeftLeg*2.0*i)*pow(cos(qLeftLeg), 2) + 4.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2)*pow(cos(qBodyOffset), 2) + 4.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*pow(sin(position.boomAngle), 2)*pow(sin(qBodyOffset), 2) + 8.0*lBoom*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) + 8.0*lBoom*lBody*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2)*cos(qBodyOffset) - 8.0*lBoom*lBody*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*sin(position.boomAngle)*sin(qBodyOffset) + 8.0*lBody*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*cos(qBodyOffset)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) - 8.0*lBody*exp(qLeftLeg*2.0*i)*sin(position.boomAngle)*sin(qBodyOffset)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) - 8.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*cos(qBodyOffset)*sin(position.boomAngle)*sin(qBodyOffset)) + 2.0*lBoom*cos(position.boomAngle)*(cos(qLeftLeg) + sin(qLeftLeg)*i) + 2.0*exp(qLeftLeg*i)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) + 2.0*lBody*cos(position.boomAngle + qBodyOffset)*(cos(qLeftLeg) + sin(qLeftLeg)*i))/(lLeftLeg + 2.0*lHip*exp(qLeftLeg*i) - lLeftLeg*exp(qLeftLeg*2.0*i)))*i;

    complexHipAngleRight = - position.boomAngle - qBodyOffset - log(-(- sqrt(2.0*pow(lBody, 2)*exp(qRightLeg*2.0*i) - 4.0*pow(lHip, 2)*exp(qRightLeg*2.0*i) - 2.0*pow(lRightLeg, 2)*exp(qRightLeg*2.0*i) + pow(lRightLeg, 2)*exp(qRightLeg*4.0*i) + 4.0*pow(toePosition.right, 2)*exp(qRightLeg*2.0*i) + pow(lRightLeg, 2) + 4.0*pow(lBoom, 2)*exp(qRightLeg*2.0*i)*pow(cos(position.boomAngle), 2) - 4.0*pow(lRightLeg, 2)*exp(qRightLeg*2.0*i)*pow(cos(qRightLeg), 2) + 2.0*pow(lBody, 2)*cos(2.0*position.boomAngle + 2.0*qBodyOffset)*exp(qRightLeg*2.0*i) + 4.0*lBoom*lBody*exp(qRightLeg*2.0*i)*cos(qBodyOffset) + 4.0*lBoom*lBody*cos(2.0*position.boomAngle + qBodyOffset)*exp(qRightLeg*2.0*i) + 8.0*lBody*exp(qRightLeg*2.0*i)*cos(position.boomAngle + qBodyOffset)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg)) + 8.0*lBoom*exp(qRightLeg*2.0*i)*cos(position.boomAngle)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg))) + 2.0*exp(qRightLeg*i)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg)) + 2.0*lBody*exp(qRightLeg*i)*cos(position.boomAngle + qBodyOffset) + 2.0*lBoom*exp(qRightLeg*i)*cos(position.boomAngle))/(- lRightLeg + 2.0*lHip*exp(qRightLeg*i) + lRightLeg*exp(qRightLeg*2.0*i)))*i;

	// TODO - Could add velocity terms but probably not worth the effort.

	// We only care about the real part, the imaginary part should be zero.
	hipAngle.left = fmod(real(complexHipAngleLeft) + 4.0*M_PI, 2.0*M_PI);
	hipAngle.right = fmod(real(complexHipAngleRight) + 4.0*M_PI, 2.0*M_PI);

	// TODO - Clamp hip angles to physical limits.
	// hipAngle.left = (hipAngle.left);
	// hipAngle.right = (hipAngle.right);
	
	return hipAngle;

} // ASCHipInverseKinematics

ORO_SERVICE_NAMED_PLUGIN(ASCHipInverseKinematics, "ascHipInverseKinematics")

} // namespace controller
} // namespace atrias
