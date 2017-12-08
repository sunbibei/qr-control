/*
 * robot_leg.h
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_LEG_MATH_LEG_H_
#define INCLUDE_ROBOT_LEG_MATH_LEG_H_

#include <foundation/label.h>
#include <robot/leg/data_leg.h>

namespace qr_control {

enum LegState {
  INVALID_STATE = -1,
  TD_STATE,
  AIR_STATE,
  N_LEG_STATE
};

class MathLeg: public DataLeg {
///! The main interface for user.
public:
  ///! The current state, enumerate the state in the LegState
  virtual LegState leg_state() = 0;
  /*!
   * @brief The forward kinematic solution for the foot link.
   * @param translation [out]  The current translation from the base frame.
   * @param quaternion  [out]  The current quaternion related to the base frame.
   */
  virtual void forwardKinematics(const EVX& angle, EVX& jnt_pos) = 0;
  /*!
   * @brief The inverse kinematic solution, given the target of foot pose.
   * @param translation [in]  The target of the translation from the base frame.
   * @param quaternion  [in]  The target of the quaternion related to the base frame.
   * @param jnt_pos     [out] The result of joint position.
   * @return Return true, if everything is OK, or return false.
   */
  virtual void inverseKinematics(const EVX& jnt_pos, EVX& angle) = 0; 

  // virtual void base_frame();
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_LEG_MATH_LEG_H_ */
