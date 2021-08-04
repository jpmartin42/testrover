#include <rover_core/rover_core.hpp>
#include <iostream>

using namespace Robot;

/* Hardware interface class RobotHardware */

// Constructor for robot
RobotHardware::RobotHardware(ros::NodeHandle &nh_input)
{
    ROS_INFO("[RobotHardware] Starting Robot");

    // Fetch parameters from ROS

    // Construct arm interface using fetched parameters

    // Create control handles for ros_control interface

    // Connect ROS services TODO

};

RobotHardware::~RobotHardware(){};

/** main() loop, core control loop **/

// Start main loop, using ROS-required variables argc and argv
int main(int argc, char **argv)
{
    const std::string node_name="robot_control_node";

    // Start ROS interface
    ros::init(argc,argv,node_name);
    ros::NodeHandle nh;

    // Create blank robot object and other variables
    RobotHardware* robot;
    std::string robotName;
    int hz;
    double rate;
    ros::Time current_time;
    ros::Duration duration;

    // TODO: Should we also create a sim here? Or just expand on the RobotHardware object
    // to be a child class of RobotHWSim instead? Will there be a difference?

    // Get robot name from ROS Parameter server
    nh.getParam("/robot_name", robot_name);

    // Construct and start the arm

    // Create rate objects for control and hardware update rates

    // Start the control loop

    // Continue loops until ROS shuts down
    ros::waitForShutdown();

    ROS_INFO("Shutting down robot.");
    return 0;
}