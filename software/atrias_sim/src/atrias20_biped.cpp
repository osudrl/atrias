#include <atrias_sim/atrias20_biped.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboControllerConnector)

// Constructor
GazeboControllerConnector::GazeboControllerConnector()
{
    legMotorGearRatio = 50;
    hipGearRatio = 64;
    legTorqueConstant = 0.060; // N*m/Amp
    hipTorqueConstant = 0.126; // N*m/Amp
    prevLeftLegAngle = 0.0;
    prevRightLegAngle = 0.0;
    prevTime = 0.0;
}

// Destructor
GazeboControllerConnector::~GazeboControllerConnector()
{
}

void GazeboControllerConnector::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Pointer to the SDF
    this->sdf = _sdf;

    // Pointer to the world
    this->world = _parent;

    // Pointer to the model
    this->model = GazeboControllerConnector::getModel("modelName");

    // Model names
    this->leftLegName = GazeboControllerConnector::getName("leftLegName");
    this->rightLegName = GazeboControllerConnector::getName("rightLegName");
    this->hipName = GazeboControllerConnector::getName("hipName");

    // Link names
    this->bodyName = GazeboControllerConnector::getName("bodyName");
    this->motorAName = GazeboControllerConnector::getName("motorAName");
    this->motorBName = GazeboControllerConnector::getName("motorBName");
    this->legAName = GazeboControllerConnector::getName("legAName");
    this->legBName = GazeboControllerConnector::getName("legBName");
    this->toeName = GazeboControllerConnector::getName("toeName");

    this->hipBodyName = GazeboControllerConnector::getName("hipBodyName");
    this->hipCenterName = GazeboControllerConnector::getName("hipCenterName");
    this->hipLeftMotorName = GazeboControllerConnector::getName("hipLeftMotorName");
    this->hipLeftMotorAttachmentName = GazeboControllerConnector::getName("hipLeftMotorAttachmentName");
    this->hipRightMotorName = GazeboControllerConnector::getName("hipRightMotorName");
    this->hipRightMotorAttachmentName = GazeboControllerConnector::getName("hipRightMotorAttachmentName");

    // Left leg link pointers
    leftLegLinks.body = this->model->GetLink(this->leftLegName + "::" + this->bodyName);
    leftLegLinks.motorA = this->model->GetLink(this->leftLegName + "::" + this->motorAName);
    leftLegLinks.motorB = this->model->GetLink(this->leftLegName + "::" + this->motorBName);
    leftLegLinks.legA = this->model->GetLink(this->leftLegName + "::" + this->legAName);
    leftLegLinks.legB = this->model->GetLink(this->leftLegName + "::" + this->legBName);
    leftLegLinks.toe = this->model->GetLink(this->leftLegName + "::" + this->toeName);

    // Right leg link pointers
    rightLegLinks.body = this->model->GetLink(this->rightLegName + "::" + this->bodyName);
    rightLegLinks.motorA = this->model->GetLink(this->rightLegName + "::" + this->motorAName);
    rightLegLinks.motorB = this->model->GetLink(this->rightLegName + "::" + this->motorBName);
    rightLegLinks.legA = this->model->GetLink(this->rightLegName + "::" + this->legAName);
    rightLegLinks.legB = this->model->GetLink(this->rightLegName + "::" + this->legBName);
    rightLegLinks.toe = this->model->GetLink(this->rightLegName + "::" + this->toeName);

    // Hip link pointers
    hipLinks.body = this->model->GetLink(this->hipName + "::" + this->hipBodyName);
    hipLinks.leftMotor = this->model->GetLink(this->hipName + "::" + this->hipLeftMotorName);
    hipLinks.leftMotorAttachment = this->model->GetLink(this->hipName + "::" + this->hipLeftMotorAttachmentName);
    hipLinks.rightMotor = this->model->GetLink(this->hipName + "::" + this->hipRightMotorName);
    hipLinks.rightMotorAttachment = this->model->GetLink(this->hipName + "::" + this->hipRightMotorAttachmentName);
    hipLinks.center = this->model->GetLink(this->hipName + "::" + this->hipCenterName);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateStart(
        boost::bind(&GazeboControllerConnector::OnUpdate, this));

    // Setup ROS
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_controller_plugin");
    ros::NodeHandle nh;

	// Tell rtOps and the controllers that we're a full biped.
	ciso.robotConfiguration = (atrias::rtOps::RobotConfiguration_t) atrias::rtOps::RobotConfiguration::BIPED_FULL;
	ciso.disableSafeties = true;

    atrias_sim_sub = nh.subscribe("atrias_controller_requests", 0, &GazeboControllerConnector::atrias_controller_callback, this);
    atrias_sim_pub = nh.advertise<atrias_msgs::robot_state>("atrias_sim_data", 10);
}


// Called by the world update start event
void GazeboControllerConnector::OnUpdate()
{
    this->lock.lock();
    // Calculate the timestep
    simTime = this->world->GetSimTime();
    simTimeTotal = simTime.sec + simTime.nsec * pow(10.0,-9);
    timestep = simTimeTotal - prevTime;
    prevTime = simTimeTotal;
    // Stuff the outgoing message
    ciso.header.stamp.sec = (uint32_t) simTime.sec;
    ciso.header.stamp.nsec = (uint32_t) simTime.nsec;

    // Left Leg
    // A-side (shin)
    this->leftLegLinks.motorA->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.lLeg.halfA.motorAngle = angle;
    ciso.lLeg.halfA.rotorAngle = angle;
    ciso.lLeg.halfA.motorVelocity = this->leftLegLinks.motorA->GetRelativeAngularVel().y;
    ciso.lLeg.halfA.rotorVelocity = this->leftLegLinks.motorA->GetRelativeAngularVel().y;

    this->leftLegLinks.legA->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.lLeg.halfA.legAngle = angle;
    ciso.lLeg.halfA.legVelocity = this->leftLegLinks.legA->GetRelativeAngularVel().y;

    // B-side (thigh)
    this->leftLegLinks.motorB->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + 3.0*M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.lLeg.halfB.motorAngle = angle;
    ciso.lLeg.halfB.rotorAngle = angle;
    ciso.lLeg.halfB.motorVelocity = this->leftLegLinks.motorB->GetRelativeAngularVel().y;
    ciso.lLeg.halfB.rotorVelocity = this->leftLegLinks.motorB->GetRelativeAngularVel().y;

    this->leftLegLinks.legB->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + 3.0*M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.lLeg.halfB.legAngle = angle;
    ciso.lLeg.halfB.legVelocity = this->leftLegLinks.legB->GetRelativeAngularVel().y;

    // Toe
    toePosZ = this->leftLegLinks.toe->GetWorldPose().pos.z;
    if (toePosZ <= 0.021)
        ciso.lLeg.toeSwitch = 1.0;
    else
        ciso.lLeg.toeSwitch = 0;

    // Right Leg
    // A-side (shin)
    this->rightLegLinks.motorA->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.rLeg.halfA.motorAngle = angle;
    ciso.rLeg.halfA.rotorAngle = angle;
    ciso.rLeg.halfA.motorVelocity = this->rightLegLinks.motorA->GetRelativeAngularVel().y;
    ciso.rLeg.halfA.rotorVelocity = this->rightLegLinks.motorA->GetRelativeAngularVel().y;

    this->rightLegLinks.legA->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.rLeg.halfA.legAngle = angle;
    ciso.rLeg.halfA.legVelocity = this->rightLegLinks.legA->GetRelativeAngularVel().y;

    // B-side (thigh)
    this->rightLegLinks.motorB->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + 3.0*M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.rLeg.halfB.motorAngle = angle;
    ciso.rLeg.halfB.rotorAngle = angle;
    ciso.rLeg.halfB.motorVelocity = this->rightLegLinks.motorB->GetRelativeAngularVel().y;
    ciso.rLeg.halfB.rotorVelocity = this->rightLegLinks.motorB->GetRelativeAngularVel().y;

    this->rightLegLinks.legB->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + 3.0*M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.rLeg.halfB.legAngle = angle;
    ciso.rLeg.halfB.legVelocity = this->rightLegLinks.legB->GetRelativeAngularVel().y;

    // Toe
    toePosZ = this->rightLegLinks.toe->GetWorldPose().pos.z;
    if (toePosZ <= 0.021)
        ciso.rLeg.toeSwitch = 1.0;
    else
        ciso.rLeg.toeSwitch = 0;

    // Hip
    // Body
    // Note: GetWorldPose returns the pose of the link's center of mass
    ciso.position.xPosition = this->hipLinks.center->GetWorldPose().pos.x;
    ciso.position.yPosition = this->hipLinks.center->GetWorldPose().pos.y;
    ciso.position.zPosition = this->hipLinks.center->GetWorldPose().pos.z;
    ciso.position.xVelocity = this->hipLinks.center->GetWorldLinearVel().x;
    ciso.position.yVelocity = this->hipLinks.center->GetWorldLinearVel().y;
    ciso.position.zVelocity = this->hipLinks.center->GetWorldLinearVel().z;

    // Leg position/velocity with respect to the hip
    hipRot = this->hipLinks.body->GetWorldPose().rot;

    motorRot = this->hipLinks.leftMotor->GetWorldPose().CoordRotationSub(hipRot);
    motorRot.GetAsAxis(axis, angle);
    angle = angle*axis.x;
    angle = wrap_angle(angle);
    ciso.lLeg.hip.legBodyAngle = angle;
    ciso.lLeg.hip.legBodyVelocity = (angle - prevLeftLegAngle) / timestep; 
    prevLeftLegAngle = angle;

    motorRot = this->hipLinks.rightMotor->GetWorldPose().CoordRotationSub(hipRot);
    motorRot.GetAsAxis(axis, angle);
    angle = angle*axis.x + M_PI;
    angle = wrap_angle(angle);
    ciso.rLeg.hip.legBodyAngle = angle;
    ciso.rLeg.hip.legBodyVelocity = (angle - prevRightLegAngle) / timestep; 
    prevRightLegAngle = angle;

    this->lock.unlock();

    // Put the robot state in the publishing queue
    atrias_sim_pub.publish(ciso);

    // Publish the robot state and get the torque requests
    ros::spinOnce();

    this->lock.lock();
    // Add the torques to the simulation
    // Left Leg
    this->leftLegLinks.motorA->AddRelativeTorque(math::Vector3(0., cosi.lLeg.motorCurrentA * legTorqueConstant * legMotorGearRatio, 0.));
    this->leftLegLinks.body->AddRelativeTorque(math::Vector3(0., -1. * cosi.lLeg.motorCurrentA * legTorqueConstant * legMotorGearRatio, 0.));
    this->leftLegLinks.motorB->AddRelativeTorque(math::Vector3(0., cosi.lLeg.motorCurrentB * legTorqueConstant * legMotorGearRatio, 0.));
    this->leftLegLinks.body->AddRelativeTorque(math::Vector3(0., -1. * cosi.lLeg.motorCurrentB * legTorqueConstant * legMotorGearRatio, 0.));

    // Right Leg
    this->rightLegLinks.motorA->AddRelativeTorque(math::Vector3(0., cosi.rLeg.motorCurrentA * legTorqueConstant * legMotorGearRatio, 0.));
    this->rightLegLinks.body->AddRelativeTorque(math::Vector3(0., -1. * cosi.rLeg.motorCurrentA * legTorqueConstant * legMotorGearRatio, 0.));
    this->rightLegLinks.motorB->AddRelativeTorque(math::Vector3(0., cosi.rLeg.motorCurrentB * legTorqueConstant * legMotorGearRatio, 0.));
    this->rightLegLinks.body->AddRelativeTorque(math::Vector3(0., -1. * cosi.rLeg.motorCurrentB * legTorqueConstant * legMotorGearRatio, 0.));

    // Hip
    this->hipLinks.rightMotor->AddRelativeTorque(math::Vector3(cosi.rLeg.motorCurrentHip * hipTorqueConstant * hipGearRatio, 0., 0.));
    this->hipLinks.body->AddRelativeTorque(math::Vector3(-1. * cosi.rLeg.motorCurrentHip * hipTorqueConstant * hipGearRatio, 0., 0.));
    this->hipLinks.leftMotor->AddRelativeTorque(math::Vector3(cosi.lLeg.motorCurrentHip * hipTorqueConstant * hipGearRatio, 0., 0.));
    this->hipLinks.body->AddRelativeTorque(math::Vector3(-1. * cosi.lLeg.motorCurrentHip * hipTorqueConstant * hipGearRatio, 0., 0.));

    this->lock.unlock();
}


void GazeboControllerConnector::atrias_controller_callback(const atrias_msgs::controller_output &temp_cosi)
{
    // Save the torque requests (Controller out, simulation in)
    cosi = temp_cosi;
}


double GazeboControllerConnector::wrap_angle(double newTheta)
{
    // Keep the angle between 2*M_PI and -2*M_PI
    if (newTheta < 0)
        theta = -1.0*fmod(-1.0*newTheta, 2*M_PI);
    else if (theta > 0)
        theta = fmod(newTheta, 2*M_PI);
    else
        theta = newTheta;

    return theta;
}

physics::ModelPtr GazeboControllerConnector::getModel(std::string requestedModelName)
{
    // Get the model name, and throw an error if we can't find it
    if (sdf->HasElement(requestedModelName))
        this->tempModelName = sdf->GetElement(requestedModelName)->GetValueString();
    else
        gzerr << "Gazebo controller wrapper plugin missing valid model name: " << requestedModelName << "\n";

    // Return the model pointer
    return this->world->GetModel(this->tempModelName);
}

std::string GazeboControllerConnector::getName(std::string requestedLinkName)
{
    // Throw an error if we can't find the link
    if ( !(sdf->HasElement(requestedLinkName)) )
        gzerr << "Gazebo controller wrapper plugin missing valid link name: " << requestedLinkName << "\n";

    // Return the link string name
    return sdf->GetElement(requestedLinkName)->GetValueString();
}
