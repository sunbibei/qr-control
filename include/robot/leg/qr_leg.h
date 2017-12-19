/*
 * qr_leg.h
 *
 *  Created on: Dec 7, 2017
 *      Author: bibei
 *  Implemented by Sampson on Dec 8, 2017
 */

#ifndef INCLUDE_ROBOT_LEG_QR_LEG_H_
#define INCLUDE_ROBOT_LEG_QR_LEG_H_

#include <robot/leg/robot_leg.h>

namespace qr_control {

class QrLeg: public RobotLeg {
public:
  QrLeg();
  virtual ~QrLeg();

///! inhert from RobotLeg
protected:
  virtual void followJntTrajectory(JntType, const Trajectory1d) override;
  virtual void followJntTrajectory(const Trajectory3d) override;
  virtual void followEefTrajectory(const Trajectory3d) override;

// inhert from MathLeg
public:
  virtual LegState leg_state() override;

protected:
  /*!
   * @brief The forward kinematic solution for the foot link.
   * @param translation [out]  The current translation from the base frame.
   * @param quaternion  [out]  The current quaternion related to the base frame.
   */
  virtual void forwardKinematics(const EVX& angle, EVX& jnt_pos) override;
  /*!
   * @brief The inverse kinematic solution, given the target of foot pose.
   * @param translation [in]  The target of the translation from the base frame.
   * @param quaternion  [in]  The target of the quaternion related to the base frame.
   * @param jnt_pos     [out] The result of joint position.
   * @return Return true, if everything is OK, or return false.
   */
  virtual void inverseKinematics(const EVX& jnt_pos, EVX& angle) override;
  // virtual bool inverse_kinematics(
  //       const Eigen::Vector3d& translation,   Eigen::Vector3d& jnt_pos) override;
  // virtual bool inverse_kinematics(
  //       const Eigen::Quaterniond& quaternion, Eigen::Vector3d& jnt_pos) override;

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
/*foot force process*/
  void setTouchdownThreshold(const double& threshold);
  void touchDownDetect();
  void touchDownConditionDetect();
  void liftDetect();
  void liftConditionDetect();

  

 private:
  class __PrivateParam* params_;
  LegState state_leg_;
  double td_threshold_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_LEG_QR_LEG_H_ */
