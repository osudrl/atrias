// Devin Koepl

#include <drl_plugins/position_body.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("position_body", PositionBody);

////////////////////////////////////////////////////////////////////////////////
// Constructor

PositionBody::PositionBody(Entity *parent)
: Controller(parent) {
    this->myParent = dynamic_cast<Model*> (this->parent);

    if (!this->myParent)
        gzthrow("PositionBody controller requires an Model as its parent");

    Param::Begin(&this->parameters);
    this->bodyNameP = new ParamT<std::string > ("bodyName", "link", 1);
    Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor

PositionBody::~PositionBody() {
    delete this->bodyNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller

void PositionBody::LoadChild(XMLConfigNode *node) {
    this->bodyNameP->Load(node);
    this->bodyName = this->bodyNameP->GetValue();

    // assert that the body by bodyName exists
    if (dynamic_cast<Body*> (this->myParent->GetBody(this->bodyName)) == NULL)
        ROS_FATAL("gazebo_ros_force plugin error: bodyName: %s does not exist\n", bodyName.c_str());

    this->body = dynamic_cast<Body*> (this->myParent->GetBody(bodyName));

    // check update rate against world physics update rate
    // should be equal or higher to guarantee the wrench applied is not "diluted"
    if (this->updatePeriod > 0 &&
            (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0 / this->updatePeriod))
        ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "position_body_interface", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    this->nh = new ros::NodeHandle();
    ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<drl_plugins::position_body_srv > (
            "position_body_srv", boost::bind(&PositionBody::position_body_gui_callback, this, _1, _2), ros::VoidPtr(), &this->queue);
    this->position_body_srv = this->nh->advertiseService(aso);

    // Initially hold the robot at a height of 1m above the origin.
    this->hold_robot = true;
    this->desired_pose = this->body->GetWorldPose();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller

void PositionBody::UpdateChild() {
    // If we are holding the robot, than apply PD force control to move it to the desired position.
    if (this->hold_robot) {
        this->lock.lock();

        Vector3 force_unconstrained = (this->desired_pose.pos - this->body->GetWorldPose().pos) * KP - this->body->GetRelativeLinearVel() * KD;
        Vector3 force_constrained;

        force_constrained.x = force_unconstrained.x * this->position_constraints.x;
        force_constrained.y = force_unconstrained.y * this->position_constraints.y;
        force_constrained.z = force_unconstrained.z * this->position_constraints.z;

        this->body->SetForce(force_constrained);

        Vector3 torque_unconstrained = (this->desired_pose.pos - this->body->GetWorldPose().pos) * KP - this->body->GetRelativeLinearVel() * KD;
        Vector3 torque_constrained;

        torque_constrained.x = torque_unconstrained.x * this->rotation_constraints.x;
        torque_constrained.y = torque_unconstrained.y * this->rotation_constraints.y;
        torque_constrained.z = torque_unconstrained.z * this->rotation_constraints.z;

        this->body->SetTorque(torque_constrained);

        this->lock.unlock();
    }
}

// Initialize the controller

void PositionBody::InitChild() {
    callback_queuethread = new boost::thread(boost::bind(&PositionBody::QueueThread, this));
}

// Set constraints
void PositionBody::SetPositionConstraints(Vector3 constraints) {
    this->rotation_constraints = constraints;
}

void PositionBody::SetRotationConstraints(Vector3 constraints) {
    this->position_constraints = constraints;
}

// Shutdown

void PositionBody::FiniChild() {
    // Callback Queue
    this->queue.clear();
    this->queue.disable();
    this->nh->shutdown();
    this->callback_queuethread->join();
}

// Simulation wants data.

bool PositionBody::position_body_gui_callback(drl_plugins::position_body_srv::Request &req, drl_plugins::position_body_srv::Response &res) {
    Pose3d actual_pose = this->body->GetWorldPose();

    // Check to see if we are supposed to pause the simulation.
    if (req.pause_simulation) {
        Simulator::Instance()->SetPaused(true);
    } else {
        Simulator::Instance()->SetPaused(false);
    }

    // Grab the request.
    //this->controller_state->val1 = req.val1;
    this->hold_robot = req.hold_robot;
    this->desired_pose.pos.x = req.desired_pose.position.x;
    this->desired_pose.pos.y = req.desired_pose.position.y;
    this->desired_pose.pos.z = req.desired_pose.position.z;

    // Pack the response.
    res.actual_pose.position.x = actual_pose.pos.x;
    res.actual_pose.position.y = actual_pose.pos.y;
    res.actual_pose.position.z = actual_pose.pos.z;

    return true;
}

// Manage the callback thread.

void PositionBody::QueueThread() {
    static const double timeout = 0.01;

    while (nh->ok()) {
        //    std::cout << "CALLING STUFF\n";
        queue.callAvailable(ros::WallDuration(timeout));
    }
}

