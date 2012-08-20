// Author: Andrew Peekema

#include <atrias_sim/inverted_pendulum.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(InvertedPendulum)

// Constructor
InvertedPendulum::InvertedPendulum()
{
	// Parameters
	pi = 3.14159265;
	l0 = 1;                                                                      
	alpha = 73.1*pi/180;
	k = 25000;
	stance = 0;                                                                  
	zVelPrev = -1;
	touchDown = 1;
	count = 0;

	// Open file for writing data                                                
	const char *homedir = getenv("HOME");
	char file_location[80];
	strcpy (file_location, homedir);
	strcat (file_location, "/GazeboInvertedPendulum.txt");
	file.open(file_location, std::ios::out);
	file << "xPos zPos stance\n";
	// Let the user know a logfile was created
	gzdbg << "Plugin logfile opened at: " << file_location << "\n";
}

// Initial plugin load
void InvertedPendulum::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	// Store the pointer to the model
	this->model = _parent;

	// Pointer to the link
	this->link = this->model->GetLink("link_name");

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateStart(
		boost::bind(&InvertedPendulum::OnUpdate, this));

	// Initial Condition (Height)
	math::Pose orig_pose = this->link->GetWorldPose();
	math::Pose pose = orig_pose;
	pose.pos.z = 0.959;
	this->link->SetWorldPose(pose);
}


// Called by the world update start event
void InvertedPendulum::OnUpdate()
{
	// Initial Condition (Velocity)
	// Note: For some reason gazebo only updates the velocity in this loop
	if (count == 0)
	{
		this->link->SetLinearVel(math::Vector3(3.699, 0, 0));
		count++;
	}

	// State
	zPos = this->link->GetWorldPose().pos.z;                                  
	xPos = this->link->GetWorldPose().pos.x;                                  
	zVel = this->link->GetWorldLinearVel().z;                                 

	// Flight
	if ( stance == 0 )
	{
		// Test stance conditions
		if (( sgn(zVel) != sgn(zVelPrev) ) && ( zPos > l0*cos(alpha) ) )
			touchDown = 1;
		if ( ( zVel < 0 ) && ( zPos <= l0*sin(alpha) ) && ( touchDown == 1 ) )
		{
			xfp = xPos + l0*cos(alpha);
			stance = 1;
		}
		else if ( zPos < l0*sin(alpha) )
			gzerr << "Unstable Solution\n";

		// No force needed
		forceVec = math::Vector3(0., 0., 0.);
	}
	// Stance
	else if ( stance == 1 )
	{
		// Test flight conditions
		l = sqrt(pow((xPos-xfp),2) + pow((zPos-0),2));
		if ( l >= l0 )
		{
			stance = 0;
			touchDown = 0;
		}
		// Calculate the spring force
		forceVec = math::Vector3(k*(l0-l)*(xPos-xfp)/l, 0., k*(l0-l)*(zPos-0)/l);
	}

	gzdbg << "xpos = " << xPos << ", zPos = " << zPos << ", stance = " << stance << ", zVel = " << zVel << ", touchDown = " << touchDown << "\n";

	// Write data to file
	file << xPos << " " << zPos << " " << stance << "\n";

	// Apply the force
	this->link->AddForce(forceVec);

	zVelPrev = zVel;
}
