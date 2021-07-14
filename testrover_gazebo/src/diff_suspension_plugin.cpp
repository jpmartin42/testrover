/*
    A simple plugin for simulating the action of a rocker suspension system between two joints.
    Based on the differential plugin from the Leo Rover open source code:
    github.com/LeoRover/leo_simulator/blob/master/leo_gazebo/src/differential_plugin.cpp
*/

#include <functional>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo {

// Create a differential suspension plugin as a child of the ModelPlugin class from Gazebo
class DiffSuspensionPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    model_ = _parent;
    world_ = model_->GetWorld();

    if (!_sdf->HasElement("jointA")) {
      ROS_ERROR(
          "No jointA element present. DiffSuspensionPlugin could not be loaded.");
      return;
    }
    joint_a_name_ = _sdf->GetElement("jointA")->Get<std::string>();

    if (!_sdf->HasElement("jointB")) {
      ROS_ERROR(
          "No jointB element present. DiffSuspensionPlugin could not be loaded.");
      return;
    }
    joint_b_name_ = _sdf->GetElement("jointB")->Get<std::string>();

    if (!_sdf->HasElement("forceConstant")) {
      ROS_ERROR("No forceConstant element present. DiffSuspensionPlugin could "
                "not be loaded.");
      return;
    }
    force_constant_ = _sdf->GetElement("forceConstant")->Get<double>();

    joint_a_ = model_->GetJoint(joint_a_name_);
    if (!joint_a_) {
      ROS_ERROR_STREAM("No joint named \""
                       << joint_a_name_
                       << "\". DiffSuspensionPlugin could not be loaded.");
      return;
    }
2
    joint_b_ = model_->GetJoint(joint_b_name_);
    if (!joint_b_) {
      ROS_ERROR_STREAM("No joint named \""
                       << joint_b_name_
                       << "\". DiffSuspensionPlugin could not be loaded.");
      return;
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DiffSuspensionPlugin::OnUpdate, this));

    ROS_INFO_STREAM("DiffSuspensionPlugin loaded! Joint A: \""
                    << joint_a_name_ << "\", Joint B: \"" << joint_b_name_
                    << "\", Force Constant: " << force_constant_);
  }

  void OnUpdate() {
    double angle_diff = joint_a_->Position() - joint_b_->Position();
    //ROS_INFO_STREAM("Angle difference = " << angle_diff);
    joint_a_->SetForce(0, -angle_diff * force_constant_);
    joint_b_->SetForce(0, angle_diff * force_constant_);
  }

private:
  std::string joint_a_name_, joint_b_name_;
  double force_constant_;

  physics::JointPtr joint_a_, joint_b_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;

  event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(DiffSuspensionPlugin)

} // end namespace gazebo