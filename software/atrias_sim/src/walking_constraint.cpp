#include <atrias_sim/walking_constraint.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(WalkingConstraint)

void WalkingConstraint::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    // Pointer to the world
    world = _parent;

    // Pointer to the model
    if (_sdf->HasElement("modelName")) {
        modelName = _sdf->GetElement("modelName")->GetValueString();
        model = world->GetModel(modelName);
    }
    else
        gzerr << "Gazebo walking constraint plugin missing valid modelName\n";

    // Pointer to the link
    if (_sdf->HasElement("linkName")) {
        linkName = _sdf->GetElement("linkName")->GetValueString();
        link = model->GetLink(linkName);
    }
    else
        gzerr << "Gazebo walking constraint plugin missing valid linkName\n";

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection = event::Events::ConnectWorldUpdateStart(
        boost::bind(&WalkingConstraint::OnUpdate, this));

    desired_pose = link->GetWorldPose();
    counter = 1.0;
    // PD Gains
    KP = 100000;
    KD = 100;
}


// Called by the world update start event
void WalkingConstraint::OnUpdate() {
    force = (desired_pose.pos - link->GetWorldPose().pos) * KP - link->GetRelativeLinearVel() * KD;
    torque = (desired_pose.rot.GetAsEuler() - link->GetWorldPose().rot.GetAsEuler()) * KP - link->GetRelativeAngularVel() * KD;
    // Gradually soften the force in the z direction
    force = math::Vector3(0.0, force[1], force[2]/counter); // x, y, z
    torque = math::Vector3(torque[0], torque[1], torque[2]); // roll, pitch, yaw

    // Set the torques
    link->SetForce(force);
    link->SetTorque(torque);

    counter = counter+0.01;
}

// vim:expandtab
