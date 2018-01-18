#include "qr_control/position_reset.h"

namespace qr_control {

PositionResetController::PositionResetController() :
  is_control_(false) {
}

PositionResetController::~PositionResetController() {

}

/**************************************************************************
   Description: initialize joints from robot_description
**************************************************************************/
bool PositionResetController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {
    int n_joints = 0;
    while(true) {
        std::string joint_name;
        std::string param_name = std::string("joint_" + std::to_string(n_joints));
        if (ros::param::get(param_name.c_str(), joint_name)) {
            std::cout << "Get Joint Name: " << joint_name;
            joint_handles_.push_back(robot->getHandle(joint_name));
        } else {
            break;
        }
        ++n_joints;
    }

    reset_sub_ = n.subscribe<std_msgs::Bool>("JointReset", 1,
       &PositionResetController::cbForReset, this);

    return true;
}

void PositionResetController::cbForReset(const std_msgs::BoolConstPtr& msg) {
  is_control_ = msg->data;
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: set model initial state
**************************************************************************/
void PositionResetController::starting(const ros::Time& time) {

}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: design state meachine: Adjust CoG <---> Switch Swing Leg
**************************************************************************/
void PositionResetController::update(const ros::Time&, const ros::Duration&) {
  if (!is_control_) return;

  is_control_ = false;
  for(auto iter = joint_handles_.begin();
      iter != joint_handles_.end(); ++iter) {
      iter->setCommand(iter->getPosition());
      // iter->setCommand(1);
      // if (abs(iter->getPosition()) >= 0.01) {
      //   iter->setCommand(0.0);
      //   is_control_ = true;
      // }
      // std::cout << iter->getPosition() << ", ";
  }
  // std::cout << std::endl;
}


} /* end for namespace qr_control */

#include <pluginlib/class_list_macros.h>

// PLUGINLIB_DECLARE_CLASS(qr_control, PositionResetController,
//                        qr_control::PositionResetController,
//                        controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(qr_control::PositionResetController, controller_interface::ControllerBase)