// Andrew Peekema

#include <drl_plugins/invertedPendulum.h>


// Register the controller using the gazebo controller factory
GZ_REGISTER_DYNAMIC_CONTROLLER("invertedPendulum", InvertedPendulum);



////////////////////////////////////////////////////////////////////////////////
// Constructor
InvertedPendulum::InvertedPendulum(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("InvertedPendulum controller requires an Model as its parent");

  Param::Begin(&this->parameters);
  this->bodyName1P = new ParamT<std::string>("bodyName1","link", 1);
  Param::End();

  // Parameters
  pi = 3.14159265;
  l0 = 1; 
  alpha = 73.1*pi/180;
  k = 25000;
  stance = 0;
  zVelPrev = -1;
  touchDown = 1;
  
  // Initial Conditions (Position, velocity)
  this->myParent->SetLinearVel(Vector3(3.699, 0, 0));

  // Open file for writing data
  file.open("/home/drl/GazeboInvertedPendulum.txt", ios::out);
  file << "xPos zPos stance\n";

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
InvertedPendulum::~InvertedPendulum()
{
  delete this->bodyName1P;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller (Get references to bodies)
void InvertedPendulum::LoadChild(XMLConfigNode *node)
{
  this->bodyName1P->Load(node);
  this->bodyName1 = this->bodyName1P->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName1)) == NULL)
    ROS_FATAL("gazebo controller (invertedPendulum) error: bodyName1: %s does not exist\n",bodyName1.c_str());

  this->body1 = dynamic_cast<Body*>(this->myParent->GetBody(bodyName1));

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR("gazebo controller (invertedPendulum) update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void InvertedPendulum::UpdateChild()
{
	//Use the command ROS_INFO for debugging
    //ROS_INFO("xPos = %f\n", xPos); 
    
    zPos = this->body1->GetWorldPose().pos.z;
    xPos = this->body1->GetWorldPose().pos.x;
    zVel = this->body1->GetWorldLinearVel().z;

    // Flight
	if ( stance == 0 ) {
        // Test stance conditions
        if (( sgn(zVel) != sgn(zVelPrev) ) && ( zPos > l0*cos(alpha) ) ) {
            touchDown = 1;
        }
        if ( ( zVel < 0 ) && ( zPos <= l0*sin(alpha) ) && ( touchDown == 1 ) ) {
            xfp = xPos + l0*cos(alpha);
            stance = 1;
        }
        else if ( zPos < l0*sin(alpha) ) {
            ROS_ERROR("Unstable Solution");
        }
        // No force needed
		force_vec = Vector3(0., 0., 0.);
	}
    // Stance
    else if ( stance == 1 ) {
        // Test flight conditions
        l = sqrt(pow((xPos-xfp),2) + pow((zPos-0),2));
        if ( l >= l0 ) {
            stance = 0;
            touchDown = 0;
        }
        // Calculate the spring force
        force_vec = Vector3(k*(l0-l)*(xPos-xfp)/l, 0., k*(l0-l)*(zPos-0)/l);
    }
    
    ROS_INFO("xPos = %f, zPos = %f, stance = %f, zVel = %f, touchDown = %f", xPos, zPos, stance, zVel, touchDown);

    // Write data to file
    file << xPos << " " << zPos << " " << stance << "\n";

    // Apply the force
    this->body1->SetForce(force_vec);

    zVelPrev = zVel;

}
