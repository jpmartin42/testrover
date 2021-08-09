#pragma once

#include <stdexcept>
#include <thread>
#include <chrono>
#include <memory>
#include <atomic>
#include <signal.h>
#include <typeinfo>

#include <ros/ros.h>

/* 
 * This code serves as an interface for talking to rover hardware,
 * either physical or in Gazebo. The 'RobotHW' class should have
 * an instance of this interface.
 * 
 * This "interface" is a collection of relevant functions and variables
 * that every robot should have.
 * 
 * First, define 'AbstractRobot' class, which declares everything a given 
 * robot should have. This won't define or implement anything.
 * 
 * Other files will define specific robots.
 * 
 * This way, new robots with radically different architectures can have
 * their own classes. 
 * 
 * This architecture is heavily inspired by
 */

namespace Robot {

/* An abstract class for all rovers */
class RobotCore : public hardware_interface::RobotHW {
    public:

        // Constructor and destructor declarations
        RobotHardware(ros::NodeHandle &nh);
        ~RobotHardware();

        void read(const ros::Time&, const ros::Duration&) override;
        void write(const ros::Time&, const ros::Duration&) override;

    private:

        // Function to get relevant params from ROS parameter server
        bool fetchRosParameters();

        // Construct robot from joints
        void buildRobot();

        // URDF model
        urdf::Model urdf;

        // 
        RobotParams robot_params;

        // Number of joints the robot possesses
        int num_joints;

        // Name of the robot, drawn from param server
        std::string robot_name;

        // Type of physical controllers the robot uses (Gazebo sim, Elmo, AMC, etc.)
        std::string robot_type;

        // Object representing the robot.
        // Built from rover_hw package objects
        AbstractRobot* robotInfo;

        // Joint info
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_efforts;
        std::vector<double> joint_commands;

        // Relevant controllers for 
        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::VelocityJointInterface vel_joint_interface;

        // ROS nodehandle for node connection
        ros::NodeHandle nh;

        /* PUT SERVICES HERE */

        /* PUT CUSTOM MESSAGES HERE */

};

// A version of the above that also inherits the abilities of the robot in question
class RobotCoreGazebo : public RobotCore, public gazebo_ros_control::RobotHWSim
{
    /* Gazebo-specific functions to interface with simulated hardware */

    // Function to initiate the sim-software interface
    virtual bool initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_sim,
        std::vector<transmission_interface::TransmissionInfo> transmissions);

    // Function to read data from the simulation
    virtual void readSim(ros::Time time, ros::Duration period);

    // Function to write to the simulation
    virtual void writeSim(ros::Time time, ros::Duration period);
    // Function to activate/deactivate an emergency stop
    virtual void eStopActive(const bool active);
};

} // end namespace Rover