#include <atrias_sim/atrias20_biped.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ControllerWrapper)

// Constructor
ControllerWrapper::ControllerWrapper()
{
    legMotorGearRatio = 50;
    hipMotorGearRatio = 30;  // TODO: Change this to the actual value
    legTorqueConstant = 0.121; // N*m/Amp
    hipTorqueConstant = 0.184; // N*m/Amp
}

// Destructor
ControllerWrapper::~ControllerWrapper()
{
}

void ControllerWrapper::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Pointer to the world
    this->world = _parent;

    // Pointer to the model
    this->model = ControllerWrapper::getModel("modelName");

    // Model names
    this->leftLegName = ControllerWrapper::getName("leftLegName");
    this->rightLegName = ControllerWrapper::getName("rightLegName");
    this->hipName = ControllerWrapper::getName("hipName");

    // Link names
    this->bodyName = ControllerWrapper::getName("bodyName");
    this->motorAName = ControllerWrapper::getName("motorAName");
    this->motorBName = ControllerWrapper::getName("motorBName");
    this->legAName = ControllerWrapper::getName("legAName");
    this->legBName = ControllerWrapper::getName("legBName");
    this->toeName = ControllerWrapper::getName("toeName");
    this->hipBodyName = ControllerWrapper::getName("hipBodyName");
    this->hipLeftMotorName = ControllerWrapper::getName("hipLeftMotorName");
    this->hipRightMotorName = ControllerWrapper::getName("hipRightMotorName");

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
    hipLinks.rightMotor = this->model->GetLink(this->hipName + "::" + this->hipRightMotorName);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateStart(
        boost::bind(&ControllerWrapper::OnUpdate, this));

    // Setup ROS
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_controller_plugin");
    ros::NodeHandle nh;

    atrias_sim_sub = nh.subscribe("atrias_controller_requests", 0, &ControllerWrapper::atrias_controller_callback, this);
    atrias_sim_pub = nh.advertise<atrias_msgs::robot_state>("atrias_sim_data", 10);
}


// Called by the world update start event
void ControllerWrapper::OnUpdate()
{
    this->lock.lock();
    // Stuff the outgoing message
    simTime = this->world->GetSimTime();
    ciso.header.stamp.sec = (uint32_t) simTime.sec;
    ciso.header.stamp.nsec = (uint32_t) simTime.nsec;

    // A-side (shin)
    this->motorA->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.lLeg.halfA.motorAngle = angle;
    ciso.lLeg.halfA.motorVelocity = this->motorA->GetRelativeAngularVel().y;

    this->legA->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.lLeg.halfA.legAngle = angle;
    ciso.lLeg.halfA.legVelocity = this->legA->GetRelativeAngularVel().y;

    // B-side (thigh)
    this->motorB->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + 3.0*M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.lLeg.halfB.motorAngle = angle;
    ciso.lLeg.halfB.motorVelocity = this->motorB->GetRelativeAngularVel().y;

    this->legB->GetRelativePose().rot.GetAsAxis(axis, angle);
    angle = angle*axis.y + 3.0*M_PI/4.0;
    angle = wrap_angle(angle);
    ciso.lLeg.halfB.legAngle = angle;
    ciso.lLeg.halfB.legVelocity = this->legB->GetRelativeAngularVel().y;

    ciso.position.xPosition = this->body->GetWorldPose().pos.x;
    ciso.position.yPosition = this->body->GetWorldPose().pos.y;
    ciso.position.zPosition = this->body->GetWorldPose().pos.z;
    ciso.position.xVelocity = this->body->GetWorldLinearVel().x;
    ciso.position.yVelocity = this->body->GetWorldLinearVel().y;
    ciso.position.zVelocity = this->body->GetWorldLinearVel().z;

    toePosZ = this->toe->GetWorldPose().pos.z;
    if (toePosZ <= 0.021)
        ciso.lLeg.toeSwitch = 1.0;
    else
        ciso.lLeg.toeSwitch = 0;

    this->lock.unlock();

    // Put the robot state in the publishing queue
    atrias_sim_pub.publish(ciso);

    // Publish the robot state and get the torque requests
    ros::spinOnce();

    this->lock.lock();
    // Add the torques to the simulation
    this->motorA->AddRelativeTorque(math::Vector3(0., cosi.lLeg.motorCurrentA * legTorqueConstant * legGearRatio, 0.));
    this->body->AddRelativeTorque(math::Vector3(0., -1. * cosi.lLeg.motorCurrentA * legTorqueConstant * legGearRatio, 0.));
    this->motorB->AddRelativeTorque(math::Vector3(0., cosi.lLeg.motorCurrentB * legTorqueConstant * legGearRatio, 0.));
    this->body->AddRelativeTorque(math::Vector3(0., -1. * cosi.lLeg.motorCurrentB * legTorqueConstant * legGearRatio, 0.));

    this->lock.unlock();
}


void ControllerWrapper::atrias_controller_callback(const atrias_msgs::controller_output &temp_cosi)
{
    // Save the torque requests (Controller out, simulation in)
    cosi = temp_cosi;
}


double ControllerWrapper::wrap_angle(double newTheta)
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

physics::ModelPtr ControllerWrapper::getModel(std::string requestedModelName);
{
    // Get the model, and throw an error if we can't find it
    if (_sdf->HasElement(requestedModelName))
    {
        this->tempModelName = _sdf->GetElement(requestedModelName)->GetValueString();
        this->tempModel = this->world->GetModel(this->tempModelName);
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid model name: " << requestedModelName << "\n";

    return this->tempModel;
}

std::string ControllerWrapper::getName(std::string requestedLinkName);
{
    // Get the link, and throw an error if we can't find it
    if (_sdf->HasElement(requestedLinkName))
    {
        this->tempLinkName = _sdf->GetElement(requestedLinkName)->GetValueString();
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid link name: " << requestedLinkName << "\n";

    return this->tempLinkName;
}
