/*
 * qr_leg.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: bibei
 */

#include <robot/leg/qr_leg.h>

namespace qr_control {

QrLeg::QrLeg() {
  // TODO Auto-generated constructor stub

}

QrLeg::~QrLeg() {
  // TODO Auto-generated destructor stub
}

void QrLeg::followJntTrajectory(JntType, const Trajectory1d) {
  LOG_ERROR << "Call the 'followJntTrajectory' which has does not complemented.";
}

void QrLeg::followJntTrajectory(const Trajectory3d) {
  LOG_ERROR << "Call the 'followJntTrajectory' which has does not complemented.";
}

void QrLeg::followEefTrajectory(const Trajectory3d) {
  LOG_ERROR << "Call the 'followJntTrajectory' which has does not complemented.";
}

LegState QrLeg::leg_state() {
  LOG_ERROR << "Call the 'leg_state' which has does not complemented.";
  return LegState::INVALID_LEG_STATE;
}
/*!
 * @brief The forward kinematic solution for the foot link.
 * @param translation [out]  The current translation from the base frame.
 * @param quaternion  [out]  The current quaternion related to the base frame.
 */
void QrLeg::forward_kinematics(
    Eigen::Vector3d& translation, Eigen::Quaterniond& quaternion) {
  LOG_ERROR << "Call the 'forward_kinematics' which has does not complemented.";
}
/*!
 * @brief The inverse kinematic solution, given the target of foot pose.
 * @param translation [in]  The target of the translation from the base frame.
 * @param quaternion  [in]  The target of the quaternion related to the base frame.
 * @param jnt_pos     [out] The result of joint position.
 * @return Return true, if everything is OK, or return false.
 */
bool QrLeg::inverse_kinematics(
    const Eigen::Vector3d& translation, const Eigen::Quaterniond& quaternion,
    Eigen::Vector3d& jnt_pos) {
  LOG_ERROR << "Call the 'inverse_kinematics' which has does not complemented.";
  return false;
}

bool QrLeg::inverse_kinematics(
      const Eigen::Vector3d& translation,   Eigen::Vector3d& jnt_pos) {
  LOG_ERROR << "Call the 'inverse_kinematics' which has does not complemented.";
  return false;
}

bool QrLeg::inverse_kinematics(
      const Eigen::Quaterniond& quaternion, Eigen::Vector3d& jnt_pos) {
  LOG_ERROR << "Call the 'inverse_kinematics' which has does not complemented.";
  return false;
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::QrLeg, Label)
