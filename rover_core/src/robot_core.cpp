#include <rover_core/rover_core.hpp>
#include <iostream>

using namespace Robot;

/* Hardware interface class RobotCore */

// Constructor for robot
RobotCore::RobotCore(ros::NodeHandle &nh_input)
{
    ROS_INFO("[RobotCore] Starting Robot");

    // Fetch parameters from ROS and allocate them to different functions

    // Construct arm interface using fetched parameters

    // Create control handles for ros_control interface

    // Connect ROS services TODO

};

// Destructor. Doesn't need to do anything fancy right now
RobotCore::~RobotCore(){};

// Function to get necessary parameters from ROS server
bool RobotCore::fetchRosParameters() {

    // Different robots have different joints, connections, etc. This function gets
    // these parameters from ROS (uploaded during .launch files), which are then allocated
    // to relevant variables

    bool fetch = ROSParamHelper::fetchRosParameters(
                nh, robot_params, urdf);

    robot_name = robot_params.robot_name;
    
    num_joints = robot_params.num_joints;

    // joints = robot_params.joints;

    return fetch;
}

/** main() loop, core control loop **/

// Start main loop, using ROS-required variables argc and argv
int main(int argc, char **argv)
{
    const std::string node_name="robot_control_node";

    // Start ROS interface
    ros::init(argc,argv,node_name);
    ros::NodeHandle nh;

    // Create blank robot object and other variables
    RobotCore* robot;
    std::string robotName;
    int hz;
    double rate;
    ros::Time current_time;
    ros::Duration duration;

    // TODO: Should we also create a sim here? Or just expand on the RobotCore object
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