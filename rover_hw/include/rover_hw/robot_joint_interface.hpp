/* This file serves to provide a joint from which robots are assembled.
 * Future development of joints (i.e. if joints have brakes, extra sensors,
 * etc.) should add more data to this section
 * 
 * Many of the functions here are virtual, meaning they can (and should) be
 * overwritten by child classes. If a joint is created, 
*/

#pragma once

#include <ros/ros.h>

#include <string>
#include <stdint.h>
#include <stdexcept>
#include <stdio.h>
#include <random>

#include <rover_hw/abstract_robot.hpp>

namespace Robot {

class RobotJoint
{
public:
    // Destructor
    virtual ~RobotJoint(){};

    /* Getter functions */
    virtual double getJointAngle() = 0;
    virtual double getJointVelocity() = 0;
    virtual double getJointEffort() = 0;
    
    // Sets the desired motor velocity, usually specified in rad/sec. Defined in child functions
    virtual bool setJointVelocity(double vel, const ros::Time &time, const ros::Duration &period) = 0;

// "Protected" qualities are the same as "private", but they can be
// modified by inherited classes.
protected:

    /* Command functions */

    // String identifier of joint
    std::string jointName;

    // current joint angle
    double jointAngle;

    // current velocity
    double jointVelocity;

    // current effort
    double jointEffort;

    // Home position of drive
    double home;

};

} // end namespace Robot