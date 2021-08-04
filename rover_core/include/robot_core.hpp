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
class RobotHardware : public hardware_interface::RobotHW {
    public:

        // Constructor and destructor declarations
        RobotHardware(ros::NodeHandle &nh);
        ~RobotHardware();

    private:

        // number of joints
        uint8_t num_joints;

};

} // end namespace Rover