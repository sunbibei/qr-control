/*
 * qr_leg.h
 *
 *  Created on: Dec 7, 2017
 *      Author: bibei
 *  Implemented by Sampson on Dec 8, 2017
 */

#ifndef INCLUDE_ROBOT_LEG_QR_LEG_H_
#define INCLUDE_ROBOT_LEG_QR_LEG_H_

#include "robot/leg/robot_leg.h"
#include <foundation/utf.h>

namespace qr_control {

class QrLeg: public RobotLeg {
public:
  QrLeg();
  virtual bool init() override;

  virtual ~QrLeg();

///! inherit from RobotLeg
protected:
  virtual void followJntTrajectory(JntType, const Traj1dSp&) override;
  virtual void followJntTrajectory(const Traj3dSp&) override;
  virtual void followEefTrajectory(const Traj3dSp&) override;

// inherit from MathLeg
public:
  ///! Offer the interface to change the touch down threshold in the runtime.
  void setForceThreshold(double);
  ///! get the current leg state.
  virtual LegState leg_state() override;

public:
  /*!
   * @brief The forward kinematic solution for the position of foot link.
   * @param translation [out]  The current translation from the base frame.
   * @param quaternion  [out]  The current quaternion related to the base frame.
   */
  // virtual void fk(Eigen::Vector3d&, Eigen::Quaterniond&) override;
  // virtual void fk(Eigen::Quaterniond&) override;
  virtual void fk(Eigen::Vector3d&)    override;
  /*!
   * @brief The inverse kinematic solution, given the target of foot pose.
   * @param translation [in]  The target of the translation from the base frame.
   * @param quaternion  [in]  The target of the quaternion related to the base frame.
   * @param jnt_pos     [out] The result of joint position.
   * @return Return true, if everything is OK, or return false.
   */
  // virtual void inverseKinematics(const EVX& jnt_pos, EVX& angle) override;
  // virtual void ik(const Eigen::Vector3d&, const Eigen::Quaterniond&, EVX& angle) override;
  virtual void ik(const Eigen::Vector3d&,   EVX& angle)  override;
  // virtual void ik(const Eigen::Quaterniond&, EVX& angle) override;
  /*！
   * @brief The forward statics solution for the foot link.
   * @param force [out] The current force output from the foot link.
   */
  virtual void fs(Eigen::Vector3d&) override;
  /*!
   * @brief The inverse statics sulution, given the target force from foot.
   * @param forces [in]  The target of the force from foot.
   * @param torque [out] The result of joint torque.
   */
  virtual void is(const Eigen::Vector3d&, Eigen::VectorXd&) override;
  /*!
   * @brief The current Jacobian matrix with given joint position.
   */
  virtual void Jacobian(Eigen::Matrix3Xd&) override;
///! Offer some convenient interfaces for user.
public:
  ///! The position of LegType::YAW joint.
  double yaw()  const { return joint_position_const_ref()(JntType::YAW);  }
  ///! The position of LegType::HIP joint.
  double hip()  const { return joint_position_const_ref()(JntType::HIP);  }
  ///! The position of LegType::KNEE joint.
  double knee() const { return joint_position_const_ref()(JntType::KNEE); }

protected:
  double               td_thres_;
  class QrLegTopology* topology_;

public:
/*model property*/
  void printDH();
  EV3 getHipPostion(const EV3& a);
  EV3 getKneePostion(const EV3& a);
  EMX getTransMatrixT01(const EV3& a);
  EMX getTransMatrixT12(const EV3& a);
  EMX getTransMatrixT23(const EV3& a);
  EMX getTransMatrixT34(const EV3& a);
  EV3 jointVelToFoot(const EV3& joint_pos, const EV3& joint_vel);
  EV3 footVelToJoint(const EV3& joint_pos, const EV3& foot_vel);
  bool getJacobMatrix(const EV3& a, EM3& JacobMatrix, EM3& inverseJacobMatrix);

};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_LEG_QR_LEG_H_ */
