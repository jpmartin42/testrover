// Relevant parameters for the whole robot and its joints

#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <map>

namespace Robot {

// A simple structure encompassing all the components of the robot
class RobotParameters {

public:
    std::string robot_name;
    std::string robot_type;

    int num_joints;

    std::vector<JointInformation*> joints;
};

// Structure for containing extra joint information - not current position, but limits
struct JointInformation {
    std::string name;
    std::string type;

    uint8_t nodeId;

    // home position
    double home;

    double torque_limits;

    // PID gains
    double kp;
    double ki;
    double kd;
}


} // end namespace Robot