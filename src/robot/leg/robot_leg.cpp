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
    break;
  case TargetType::EEF_TRAJ:
    followEefTrajectory(eef_traj_target_);
    break;
  default:
    break;
  }
}

void RobotLeg::execut(const JntTarget& p) {
  if (JntDataType::POS != p.jnt_cmd_type) {
    LOG_ERROR << "Only Support Position Mode!";
    joint_command_ref()(p.jnt_type) = p.target;
  }
}

void RobotLeg::execut(const EVX& p) {
  EVX _jnt_cmd;
  inverseKinematics(p, _jnt_cmd);
  for (int i = 0; i < N_JNTS; ++i)
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

void RobotLeg::eefPositionTarget(const EV3& t) {
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

const EV3&    RobotLeg::eefPositionTarget() {
  return eef_xyz_target_;
}

const Trajectory3d&   RobotLeg::eefTrajectoryTarget() {
  return eef_traj_target_;
}

void RobotLeg:: eefOriPos(EVX& _rpy, EVX& _xyz)
{
  _rpy = joint_position();
  forwardKinematics(_rpy, _xyz);
}

} /* namespace qr_control */
