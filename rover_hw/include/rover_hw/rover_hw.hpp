#pragma once

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

namespace Rover {

/* An abstract class for all rovers */
class RoverRobot : public hardware_interface::RobotHW {
    public:

    private:

        // number of joints
        uint8_t num_joints;

};

} end namespace Rover