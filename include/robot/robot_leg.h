/*
 * robot_leg.h
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_ROBOT_LEG_H_
#define INCLUDE_ROBOT_ROBOT_LEG_H_

#include <system/foundation/label.h>
#include <Eigen/Dense>

namespace qr_control {

enum LegState {
  INVALID_STATE = -1,
  TD_STATE,
  AIR_STATE,
  N_LEG_STATE
};

class RobotLeg: public Label {
public:
  RobotLeg();
  virtual bool init() override;
  virtual ~RobotLeg();

///! The main interface for user.
public:
  ///! The current state, enumerate the state in the LegState
  virtual LegState leg_state() = 0;
  /*!
   * @brief The forward kinematic solution for the foot link.
   * @param translation [out]  The current translation from the base frame.
   * @param quaternion  [out]  The current quaternion related to the base frame.
   */
  virtual void forward_kinematics(
      Eigen::Vector3d& translation, Eigen::Quaterniond& quaternion) = 0;
  /*!
   * @brief The inverse kinematic solution, given the target of foot pose.
   * @param translation [in]  The target of the translation from the base frame.
   * @param quaternion  [in]  The target of the quaternion related to the base frame.
   * @param jnt_pos     [out] The result of joint position.
   * @return Return true, if everything is OK, or return false.
   */
  virtual bool inverse_kinematics(
      const Eigen::Vector3d& translation, const Eigen::Quaterniond& quaternion,
      Eigen::Vector3d& jnt_pos) = 0;

protected:
  ///! The type of this leg, reference to utf.h
  JntType                jnt_type_;
  ///! The data of the foot's force sensor.
  const short*           foot_force_;
  ///! The vector of joint position which size is 3.
  const Eigen::VectorXd* jnt_pos_;
  ///! The vector of joint command.
  Eigen::VectorXd*       jnt_cmd_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_ROBOT_LEG_H_ */
