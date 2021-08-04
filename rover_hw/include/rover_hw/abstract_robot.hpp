/* This file serves as a base for creating robots from multiple joints.
 * 
 * Many of the functions here are virtual, meaning they can (and should) be
 * overwritten by child classes.
*/


#include <ros/ros.h>

namespace Robot {

// Broad class to be modified for each joint type.
class AbstractRobot
{
public:

    virtual ~AbstractRobot(){};

    

protected:

    // number of joints
    uint8_t num_joints;

};

} // end namespace Rover