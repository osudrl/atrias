// Devin Koepl

#include <atrias_sim/all_in_one_controller_wrapper.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("all_in_one_controller_wrapper", AllInOneControllerWrapper);

////////////////////////////////////////////////////////////////////////////////
// Constructor

AllInOneControllerWrapper::AllInOneControllerWrapper(Entity *parent)
: Controller(parent)
{
    //	PhysicsEngine::InitForThread();

    this->myParent = dynamic_cast<Model*> (this->parent);

    if (!this->myParent)
        gzthrow("ControllerWrapper requires a Model as its parent");

    Param::Begin(&this->parameters);
    this->bodyNameP = new ParamT<std::string > ("bodyName", "link", 1);
    this->motorANameP = new ParamT<std::string > ("motorAName", "link", 1);
    this->motorBNameP = new ParamT<std::string > ("motorBName", "link", 1);
    this->legANameP = new ParamT<std::string > ("legAName", "link", 1);
    this->legBNameP = new ParamT<std::string > ("legBName", "link", 1);
    Param::End();

    this->controllerInput = new ControllerInput();
    this->controllerOutput = new ControllerOutput();
    this->controllerStatus = new ControllerState();
    this->controller_data = new ControllerData();

    this->controllerStatus->state = CSSM_STATE_INIT;

    atrias_data_publish_counter = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor

AllInOneControllerWrapper::~AllInOneControllerWrapper()
{
    controllerStatus->state = CSSM_STATE_FINI;
    //control_switcher_state_machine(controller_input, controller_output, controller_state, controller_data);

    delete this->bodyNameP;
    delete this->motorANameP;
    delete this->motorBNameP;
    delete this->legANameP;
    delete this->legBNameP;

    delete this->controllerInput;
    delete this->controllerOutput;
    delete this->controllerStatus;
    delete this->controller_data;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller

void AllInOneControllerWrapper::LoadChild(XMLConfigNode *node)
{
    //	Simulator::Instance()->SetPaused(true);

    this->bodyNameP->Load(node);
    this->bodyName = this->bodyNameP->GetValue();

    this->motorANameP->Load(node);
    this->motorAName = this->motorANameP->GetValue();

    this->motorBNameP->Load(node);
    this->motorBName = this->motorBNameP->GetValue();

    this->legANameP->Load(node);
    this->legAName = this->legANameP->GetValue();

    this->legBNameP->Load(node);
    this->legBName = this->legBNameP->GetValue();

    // assert that the body by bodyName exists
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->bodyName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: bodyName: %s does not exist\n", bodyName.c_str());
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->motorAName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: motorAName: %s does not exist\n", motorAName.c_str());
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->motorBName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: motorBName: %s does not exist\n", motorBName.c_str());
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->legAName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: legAName: %s does not exist\n", legAName.c_str());
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->legBName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: legBName: %s does not exist\n", legBName.c_str());

    this->body = dynamic_cast<Body*> (this->myParent->GetBody(bodyName));
    this->motorA = dynamic_cast<Body*> (this->myParent->GetBody(motorAName));
    this->motorB = dynamic_cast<Body*> (this->myParent->GetBody(motorBName));
    this->legA = dynamic_cast<Body*> (this->myParent->GetBody(legAName));
    this->legB = dynamic_cast<Body*> (this->myParent->GetBody(legBName));

    // check update rate against world physics update rate
    // should be equal or higher to guarantee the wrench applied is not "diluted"
    if (this->updatePeriod > 0 &&
            (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0 / this->updatePeriod))
        ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

    this->generate_controller_input();
    control_switcher_state_machine(controllerInput, controllerOutput, controllerStatus, controller_data);
    int argc = 0;
    char** argv = NULL;
    //ros::init(argc, argv, "gui_interface", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    ros::init(argc, argv, "gui_interface");
    ros::NodeHandle nh;

    atrias_sim_sub = nh.subscribe("atrias_controller_requests", 0, &AllInOneControllerWrapper::atrias_gui_callback, this);
    atrias_sim_pub = nh.advertise<atrias_msgs::atrias_data>("atrias_data_50_hz", 10);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller

void AllInOneControllerWrapper::UpdateChild()
{
    this->lock.lock();

    this->generate_controller_input();

    this->poke_ros();

    control_switcher_state_machine(this->controllerInput, this->controllerOutput, this->controllerStatus, this->controller_data);

    this->controllerOutput->motor_torqueA = CLAMP(this->controllerOutput->motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ);
    this->controllerOutput->motor_torqueB = CLAMP(this->controllerOutput->motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ);

    this->motorA->SetTorque(Vector3(0., -this->controllerOutput->motor_torqueA * GEAR_RATIO, 0.));
    this->body->SetTorque(Vector3(0., this->controllerOutput->motor_torqueA * GEAR_RATIO, 0.));
    this->motorB->SetTorque(Vector3(0., -this->controllerOutput->motor_torqueB * GEAR_RATIO, 0.));
    this->body->SetTorque(Vector3(0., this->controllerOutput->motor_torqueB * GEAR_RATIO, 0.));

    this->lock.unlock();
}

void AllInOneControllerWrapper::poke_ros() {
    ros::spinOnce();

    if (this->controllerStatus->state == CSSM_STATE_ENABLED)
    {
        cs.status = CMD_RUN;
    }
    else
    {
        cs.status = CMD_DISABLE;
    }

    cs.time = (uint32_t) Simulator::Instance()->GetSimTime().Double();
    cs.body_angle = this->controllerInput->body_angle;
    cs.motor_angleA = this->controllerInput->motor_angleA;
    cs.motor_angleB = this->controllerInput->motor_angleB;
    cs.leg_angleA = this->controllerInput->leg_angleA;
    cs.leg_angleB = this->controllerInput->leg_angleB;
    cs.motor_torqueA = this->controllerOutput->motor_torqueA;
    cs.motor_torqueB = this->controllerOutput->motor_torqueB;
    cs.xPosition = this->controllerInput->xPosition;
    cs.yPosition = this->controllerInput->yPosition;
    cs.zPosition = this->controllerInput->zPosition;
    cs.xVelocity = this->controllerInput->xVelocity;
    cs.yVelocity = this->controllerInput->yVelocity;
    cs.zVelocity = this->controllerInput->zVelocity;

    // Clone data from the gui into the controller's data memory.
    for (int i = 0; i < SIZE_OF_CONTROLLER_STATE_DATA; i++)
    {
        cs.control_state[i] = this->controllerStatus->data[i];
    }

    if ((atrias_data_publish_counter % 20) == 0) {   // 50 Hz publish rate.
        atrias_sim_pub.publish(cs);
    }
    atrias_data_publish_counter = (atrias_data_publish_counter + 1) % 1000;
}

void AllInOneControllerWrapper::generate_controller_input()
{
    double angle;
    Vector3 axis;

    this->body->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controllerInput->body_angle = angle * axis.y;
    this->motorA->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controllerInput->motor_angleA = -angle * axis.y + 3. * PI / 4. - PI;
    //ROS_INFO("maA = %.3f", angle);
    this->motorB->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controllerInput->motor_angleB = -angle * axis.y + PI / 4. + PI;
    //ROS_INFO("maB = %.3f", angle);
    this->legA->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controllerInput->leg_angleA = -angle * axis.y + 3. * PI / 4. - PI;
    //ROS_INFO("laA = %.3f", angle);
    this->legB->GetWorldPose().rot.GetAsAxis(axis, angle);
    this->controllerInput->leg_angleB = -angle * axis.y + PI / 4. + PI;
    //ROS_INFO("laB = %.3f\n\n", angle);

    this->controllerInput->xPosition = this->body->GetWorldPose().pos.x;
    this->controllerInput->yPosition = this->body->GetWorldPose().pos.y;
    this->controllerInput->zPosition = this->body->GetWorldPose().pos.z;

    this->controllerInput->xVelocity = this->body->GetWorldLinearVel().x;
    this->controllerInput->yVelocity = this->body->GetWorldLinearVel().y;
    this->controllerInput->zVelocity = this->body->GetWorldLinearVel().z;

    this->controllerInput->motor_velocityA = -this->motorA->GetWorldAngularVel().y;
    this->controllerInput->motor_velocityB = -this->motorB->GetWorldAngularVel().y;
    this->controllerInput->leg_velocityA = -this->legA->GetWorldAngularVel().y;
    this->controllerInput->leg_velocityB = -this->legB->GetWorldAngularVel().y;
}

void AllInOneControllerWrapper::atrias_gui_callback(const atrias_msgs::atrias_controller_requests &cr)
{
    // Grab the request.
    this->controller_data->command = cr.command;
    this->controller_data->controller_requested = cr.controller_requested;

    // Clone data from the gui into the controller's data memory.
    for (int i=0; i<SIZE_OF_CONTROLLER_DATA; i++)
    {
        this->controller_data->data[i] = cr.control_data[i];
    }
}

