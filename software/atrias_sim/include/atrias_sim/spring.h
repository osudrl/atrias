// Author: Andrew Peekema

#ifndef __SPRING_H__
#define __SPRING_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo
{
    class Spring : public ModelPlugin
    {
        // Constructor
        public: Spring();

        // Functions
        public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        public: void OnUpdate();

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        private: physics::ModelPtr model;
        private: physics::LinkPtr link1;
        private: std::string link1Name;
        private: physics::LinkPtr link2;
        private: std::string link2Name;

        private: math::Vector3 axis, torque_vec;

        private: double rotationalStiffness, angle, angle1, angle2, diff, torque;

    };
}

#endif
