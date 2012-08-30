#include <atrias_sim/atrias20_leg.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ControllerWrapper)

ControllerWrapper::ControllerWrapper()
{
    gearRatio = 50;
    legTorqueConstant = 0.121; // N*m/Amp
    hipTorqueConstant = 0.184; // N*m/Amp
}

ControllerWrapper::~ControllerWrapper()
{
}

void ControllerWrapper::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Pointer to the world
    this->world = _parent;

    // Pointer to the model
    if (_sdf->HasElement("modelName"))
    {
        this->modelName = _sdf->GetElement("modelName")->GetValueString();
        this->model = this->world->GetModel(this->modelName);
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid modelName\n";

    // Link pointers
    if (_sdf->HasElement("bodyName"))
    {
        this->bodyName = _sdf->GetElement("bodyName")->GetValueString();
        this->body = this->model->GetLink(this->bodyName);
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid bodyName\n";

    if (_sdf->HasElement("motorAName"))
    {
        this->motorAName = _sdf->GetElement("motorAName")->GetValueString();
        this->motorA = this->model->GetLink(this->motorAName);
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid motorAName\n";

    if (_sdf->HasElement("motorBName"))
    {
        this->motorBName = _sdf->GetElement("motorBName")->GetValueString();
        this->motorB = this->model->GetLink(this->motorBName);
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid motorBName\n";

    if (_sdf->HasElement("legAName"))
    {
        this->legAName = _sdf->GetElement("legAName")->GetValueString();
        this->legA = this->model->GetLink(this->legAName);
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid legAName\n";

    if (_sdf->HasElement("legBName"))
    {
        this->legBName = _sdf->GetElement("legBName")->GetValueString();
        this->legB = this->model->GetLink(this->legBName);
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid legBName\n";

    if (_sdf->HasElement("toeName"))
    {
        this->toeName = _sdf->GetElement("toeName")->GetValueString();
        this->toe = this->model->GetLink(this->toeName);
    }
    else
        gzerr << "Gazebo controller wrapper plugin missing valid toeName\n";

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
    ciso.lLeg.halfA.rotorAngle = angle;
    ciso.lLeg.halfA.motorVelocity = this->motorA->GetRelativeAngularVel().y;
    ciso.lLeg.halfA.rotorVelocity = this->motorA->GetRelativeAngularVel().y;

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
    ciso.lLeg.halfB.rotorAngle = angle;
    ciso.lLeg.halfB.motorVelocity = this->motorB->GetRelativeAngularVel().y;
    ciso.lLeg.halfB.rotorVelocity = this->motorB->GetRelativeAngularVel().y;

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
    this->motorA->AddRelativeTorque(math::Vector3(0., cosi.lLeg.motorCurrentA * legTorqueConstant * gearRatio, 0.));
    this->body->AddRelativeTorque(math::Vector3(0., -1. * cosi.lLeg.motorCurrentA * legTorqueConstant * gearRatio, 0.));
    this->motorB->AddRelativeTorque(math::Vector3(0., cosi.lLeg.motorCurrentB * legTorqueConstant * gearRatio, 0.));
    this->body->AddRelativeTorque(math::Vector3(0., -1. * cosi.lLeg.motorCurrentB * legTorqueConstant * gearRatio, 0.));

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
