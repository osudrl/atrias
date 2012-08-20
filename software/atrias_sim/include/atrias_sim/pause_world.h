// Author: Andrew Peekema

#ifndef __PAUSE_WORLD_H__
#define __PAUSE_WORLD_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo
{
	class PauseWorld : public WorldPlugin
	{
		// Constructor
		public: PauseWorld();

		// Functions
		public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
		public: void OnUpdate();

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		private: physics::WorldPtr world;

		private: double count;

	};
}

#endif
