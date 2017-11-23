/*
 * robot_leg.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#include <robot/robot_leg.h>

///! Just test
// class Trajectory { };

namespace qr_control {

RobotLeg::RobotLeg()
  : MathLeg(),
    jnt_traj_target_(nullptr), eef_traj_target_(nullptr),
    curr_target_(TargetType::INVALID_TARGET) {
  ;
}

RobotLeg::~RobotLeg() {
  if (jnt_traj_target_) {
    delete jnt_traj_target_;
    jnt_traj_target_ = nullptr;
  }
  if (eef_traj_target_) {
    delete eef_traj_target_;
    eef_traj_target_ = nullptr;
  }
}

/*void RobotLeg::asyncMove(bool& ret_ref) {
  ;
}*/

void RobotLeg::move() {
  switch (curr_target_) {
  case TargetType::JNT_CMD:
    execut(jnt_target_);
    break;
  case TargetType::JNT_TRAJ:
    followJntTrajectory(jnt_traj_target_);
    break;
  case TargetType::EEF_XYZ:
    execut(eef_xyz_target_);
    // inverse_kinematics();
    break;
  case TargetType::EEF_RPY:
    execut(eef_rpy_target_);
    break;
  case TargetType::EEF_TRAJ:
    followJntTrajectory(eef_traj_target_);
    break;
  default:
    break;
  }
}

void RobotLeg::execut(const JntTarget& t) {
  (*jnt_cmd_[t.jnt_cmd_type])[t.jnt_type] = t.target;
}

void RobotLeg::execut(const Eigen::Quaterniond& t) {
  Eigen::Vector3d _jnt_cmd;
  inverse_kinematics(t, _jnt_cmd);
  for (int i = 0; i < _jnt_cmd.size(); ++i)
    (*jnt_cmd_[JntCmdType::POS])[i] = _jnt_cmd[i];

}

void RobotLeg::execut(const Eigen::Vector3d& t) {
  Eigen::Vector3d _jnt_cmd;
  inverse_kinematics(t, _jnt_cmd);
  for (int i = 0; i < _jnt_cmd.size(); ++i)
    (*jnt_cmd_[JntCmdType::POS])[i] = _jnt_cmd[i];

}

///! setter methods
void RobotLeg::jointTarget(const JntTarget& t) {
  jnt_target_  = t;
  curr_target_ = TargetType::JNT_CMD;
}

void RobotLeg::jointTarget(JntType jnt_type, JntCmdType jnt_cmd_type, double target) {
  jnt_target_.jnt_cmd_type = jnt_cmd_type;
  jnt_target_.jnt_type     = jnt_type;
  jnt_target_.target       = target;
  curr_target_             = TargetType::JNT_CMD;
}

void RobotLeg::jointTrajectoryTarget(const Trajectory&) {
  // TODO
}

void RobotLeg::eefOrientationTarget(const Eigen::Quaterniond& t) {
  eef_rpy_target_ = t;
  curr_target_    = TargetType::EEF_RPY;
}

void RobotLeg::eefPositionTarget(const Eigen::Vector3d& t) {
  eef_xyz_target_ = t;
  curr_target_    = TargetType::EEF_XYZ;
}
void RobotLeg::eefTrajectoryTarget(const Trajectory& t) {
  // TODO
}

///! getter methods
const JntTarget& RobotLeg::jointTarget() {
  return jnt_target_;
}

const Trajectory&   RobotLeg::jointTrajectoryTarget() {
  return *jnt_traj_target_;
}

const Eigen::Quaterniond& RobotLeg::eefOrientationTarget() {
  return eef_rpy_target_;
}

const Eigen::Vector3d&    RobotLeg::eefPositionTarget   () {
  return eef_xyz_target_;
}
const Trajectory&   RobotLeg::eefTrajectoryTarget       () {
  return *eef_traj_target_;
}

void RobotLeg::jointPosition(Eigen::Vector3d& _p) {
  _p = *jnt_pos_;
}

void RobotLeg::eef(Eigen::Vector3d& _xyz, Eigen::Quaterniond& _rpy) {
  forward_kinematics(_xyz, _rpy);
}

void RobotLeg::eef(Eigen::Vector3d& _xyz) {
  Eigen::Quaterniond _rpy;
  forward_kinematics(_xyz, _rpy);
}

void RobotLeg::eef(Eigen::Quaterniond& _rpy) {
  Eigen::Vector3d _xyz;
  forward_kinematics(_xyz, _rpy);
}

} /* namespace qr_control */
