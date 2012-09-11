// Author: Andrew Peekema

#ifndef __ATRIAS20_BIPED_H__
#define __ATRIAS20_BIPED_H__

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <ros/ros.h>

#include <atrias_msgs/robot_state.h>  // controller input
#include <atrias_msgs/controller_output.h>

namespace gazebo
{
class GazeboControllerConnector : public WorldPlugin
{
    public:
        // Constructor/Destructor
        GazeboControllerConnector();
        ~GazeboControllerConnector();

        // Functions
        virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
        void OnUpdate();
        void atrias_controller_callback(const atrias_msgs::controller_output &temp_cosi);
        double wrap_angle(double newTheta);
        physics::ModelPtr getModel(std::string requestedModelName);
        std::string getName(std::string requestedLinkName);

    private:
        // Function variables
        std::string tempModelName;
        double theta;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        // Model names
        std::string hipName;
        std::string leftLegName;
        std::string rightLegName;

        // Link names
        std::string bodyName;
        std::string motorAName;
        std::string motorBName;
        std::string legAName;
        std::string legBName;
        std::string toeName;
        std::string hipBodyName;
        std::string hipCenterName;
        std::string hipLeftMotorName;
        std::string hipLeftMotorAttachmentName;
        std::string hipRightMotorName;
        std::string hipRightMotorAttachmentName;

        // SDF pointer
        sdf::ElementPtr sdf;

        // World pointer
        physics::WorldPtr world;

        // Model pointer
        physics::ModelPtr model;

        // Link pointers
        struct LegLinks {
            physics::LinkPtr body;
            physics::LinkPtr motorA;
            physics::LinkPtr motorB;
            physics::LinkPtr legA;
            physics::LinkPtr legB;
            physics::LinkPtr toe;
        } leftLegLinks, rightLegLinks;

        struct HipLinks {
            physics::LinkPtr body;
            physics::LinkPtr center;
            physics::LinkPtr leftMotor;
            physics::LinkPtr leftMotorAttachment;
            physics::LinkPtr rightMotor;
            physics::LinkPtr rightMotorAttachment;
        } hipLinks;

        double angle, toePosZ;
        double legMotorGearRatio, hipGearRatio, legTorqueConstant, hipTorqueConstant;
        double prevLeftLegAngle, prevRightLegAngle;
        math::Vector3 axis, bodyForce;
        math::Quaternion hipRot, motorRot;
        common::Time simTime;
        double simTimeTotal, prevTime, timestep;

        boost::mutex lock;

        ros::Subscriber atrias_sim_sub;
        ros::Publisher atrias_sim_pub;
        atrias_msgs::robot_state ciso;  // Controller in, simulation out
        atrias_msgs::controller_output cosi;  // Controller out, simulation in

};
}

#endif
