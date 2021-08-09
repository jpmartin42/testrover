// A simple structure encompassing all the components of the robot

#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <map>

namespace Robot {

class RobotParameters {

public:
    std::string robot_name;
    std::string robot_type;

    int num_joints;

    std::vector<JointInformation*> joints;
};

} // end namespace Robot