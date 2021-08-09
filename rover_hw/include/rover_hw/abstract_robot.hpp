/* This file serves as a base for creating robots from multiple joints.
 * 
 * Many of the functions here are virtual, meaning they can (and should) be
 * overwritten by child classes.
*/
#pragma once

#include <ros/ros.h>

namespace Robot {

// Broad class to be modified for each joint type.
class AbstractRobot
{
public:

    virtual ~AbstractRobot(){};

    // Command to send joint commands to joints in order.
    // Joints are stored in a single vector.
    virtual void sendJointCommands(const std::vector<double> &commands,
                           const ros::Time &time,
                           const ros::Duration &period) = 0;

    // Read positions, velocities and efforts and copy those values
    // to provided vectors, in order.
    virtual void readJointData(std::vector<double> &positions,
                       std::vector<double> &velocities,
                       std::vector<double> &efforts) = 0;

protected:

    // number of joints
    uint8_t num_joints;

    // List of joints in the robot
    //std::vector<JointInformation*> joint_infos;
};

} // end namespace Rover