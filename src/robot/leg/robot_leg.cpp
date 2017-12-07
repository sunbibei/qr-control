/*
 * robot_leg.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#include <robot/leg/robot_leg.h>

///! Just test
// class Trajectory { };

namespace qr_control {

RobotLeg::RobotLeg()
  : MathLeg(),
    curr_target_(TargetType::INVALID_TARGET) {
  curr_target_jnt_ = JntType::UNKNOWN_JNT;
}

RobotLeg::~RobotLeg() {
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
    // Using the N_JNTS represent the all of joints.
    if (JntType::N_JNTS != curr_target_jnt_)
      followJntTrajectory(curr_target_jnt_, jnt_traj_target_);
    else
      followJntTrajectory(jnts_traj_target_);
    break;
  case TargetType::EEF_XYZ:
    execut(eef_xyz_target_);
    // inverse_kinematics();
    break;
  case TargetType::EEF_RPY:
    execut(eef_rpy_target_);
    break;
  case TargetType::EEF_TRAJ:
    followEefTrajectory(eef_traj_target_);
    break;
  default:
    break;
  }
}

void RobotLeg::execut(const JntTarget& t) {
  if (JntDataType::POS != t.jnt_cmd_type) {
    LOG_ERROR << "Only Support Position Mode!";
    joint_command_ref()(t.jnt_type) = t.target;
  }
}

void RobotLeg::execut(const Eigen::Quaterniond& t) {
  Eigen::Vector3d _jnt_cmd;
  inverse_kinematics(t, _jnt_cmd);
  for (int i = 0; i < _jnt_cmd.size(); ++i)
    joint_command_ref()(i) = _jnt_cmd[i];
}

void RobotLeg::execut(const Eigen::Vector3d& t) {
  Eigen::Vector3d _jnt_cmd;
  inverse_kinematics(t, _jnt_cmd);
  for (int i = 0; i < _jnt_cmd.size(); ++i)
    joint_command_ref()(i) = _jnt_cmd[i];

}

///! setter methods
void RobotLeg::jointTarget(const JntTarget& t) {
  jnt_target_  = t;
  curr_target_ = TargetType::JNT_CMD;
}

void RobotLeg::jointTarget(JntType jnt_type, JntDataType jnt_cmd_type, double target) {
  jnt_target_.jnt_cmd_type = jnt_cmd_type;
  jnt_target_.jnt_type     = jnt_type;
  jnt_target_.target       = target;
  curr_target_             = TargetType::JNT_CMD;
}

void RobotLeg::jointTrajectoryTarget(JntType _t, const Trajectory1d& _traj) {
  // TODO
  curr_target_jnt_ = _t;
  jnt_traj_target_ = _traj;
}

void RobotLeg::jointTrajectoryTarget(const Trajectory3d& _traj) {
  // TODO
  curr_target_jnt_ = JntType::N_JNTS;
  jnts_traj_target_ = _traj;
}

void RobotLeg::eefOrientationTarget(const Eigen::Quaterniond& t) {
  eef_rpy_target_ = t;
  curr_target_    = TargetType::EEF_RPY;
}

void RobotLeg::eefPositionTarget(const Eigen::Vector3d& t) {
  eef_xyz_target_ = t;
  curr_target_    = TargetType::EEF_XYZ;
}

void RobotLeg::eefTrajectoryTarget(const Trajectory3d& t) {
  // TODO
}

///! getter methods
const JntTarget& RobotLeg::jointTarget() {
  return jnt_target_;
}

const Trajectory1d&   RobotLeg::jointTrajectoryTarget() {
  return jnt_traj_target_;
}

const Eigen::Quaterniond& RobotLeg::eefOrientationTarget() {
  return eef_rpy_target_;
}

const Eigen::Vector3d&    RobotLeg::eefPositionTarget() {
  return eef_xyz_target_;
}

const Trajectory3d&   RobotLeg::eefTrajectoryTarget() {
  return eef_traj_target_;
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
