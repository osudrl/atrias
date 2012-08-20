// Author: Andrew Peekema

#ifndef __INVERTED_PENDULUM_WITH_FOOT_H__
#define __INVERTED_PENDULUM_WITH_FOOT_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

// For Math
#include <math.h>

// File operations
#include <iostream>
#include <fstream>

namespace gazebo
{
	class InvertedPendulum : public WorldPlugin
	{
		// Constructor
		public: InvertedPendulum();

		// Functions
		public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate();

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		private: physics::WorldPtr world;
		private: physics::ModelPtr model;
		private: physics::ModelPtr foot_model;
		private: physics::LinkPtr link;
		private: physics::LinkPtr foot_link;

		private: math::Vector3 massForceVec;
		private: math::Vector3 footForceVec;

		private: math::Pose foot_pose;

		// Controller Variables
		private: float zPos, xPos, zVel;
		private: float pi, l0, alpha, k;
		private: float F, phi, xfp, l, stance, touchDown;
		private: float footOffset, footX, footZ;
		private: int count;

		template <typename T> int sgn(T val)
		{
			return (T(0) < val) - (val < T(0));
		}

		private: std::ofstream file;

	};
}

#endif
