#ifndef __HOPPING_CONSTRAINT_H__
#define __HOPPING_CONSTRAINT_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo {

class HoppingConstraint : public WorldPlugin {
    public:
        virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
        void OnUpdate();

    private:
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        physics::WorldPtr world;
        physics::ModelPtr model;
        std::string modelName;
        physics::LinkPtr link;
        std::string linkName;

        math::Pose desired_pose;
        math::Vector3 force, torque;

        double KP, KD, counter;

};

} // namespace gazebo

#endif

// vim:expandtab
