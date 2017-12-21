#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

namespace qr_control
{
class PositionResetController :
        public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
  PositionResetController();
  ~PositionResetController();

  bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

  void cbForReset(const std_msgs::BoolConstPtr&);

private:
  std::vector<hardware_interface::JointHandle> joint_handles_;
  ros::Subscriber reset_sub_;

  bool is_control_;
};

}
