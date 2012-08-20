// Author: Andrew Peekema

#ifndef __INVERTED_PENDULUM_H__
#define __INVERTED_PENDULUM_H__

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
	class InvertedPendulum : public ModelPlugin
	{
		// Constructor
		public: InvertedPendulum();

		// Functions
		public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate();

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		private: physics::ModelPtr model;
		private: physics::LinkPtr link;
		//private: physics::JointPtr joint;

		private: math::Vector3 forceVec;

		// Controller Variables
		private: float zPos, xPos, zVel, zVelPrev;
		private: float pi, l0, alpha, k;
		private: float F, phi, xfp, l, stance, touchDown;
		private: int count;

		template <typename T> int sgn(T val)
		{
			return (T(0) < val) - (val < T(0));
		}

		private: std::ofstream file;

	};
}

#endif
