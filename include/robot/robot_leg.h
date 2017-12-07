/*
 * robot_leg.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_ROBOT_LEG_H_
#define INCLUDE_ROBOT_ROBOT_LEG_H_

#include <adt/trajectory.h>
#include <robot/math_leg.h>

namespace qr_control {

struct JntTarget {
  JntType    jnt_type;
  JntDataType jnt_cmd_type;
  double     target;
};

class RobotLeg: public MathLeg {
public:
  RobotLeg();
  virtual ~RobotLeg();

// inhert from MathLeg
protected:
  virtual LegState leg_state() override;
  /*!
   * @brief The forward kinematic solution for the foot link.
   * @param translation [out]  The current translation from the base frame.
   * @param quaternion  [out]  The current quaternion related to the base frame.
   */
  virtual void forward_kinematics(
      Eigen::Vector3d& translation, Eigen::Quaterniond& quaternion) override;
  /*!
   * @brief The inverse kinematic solution, given the target of foot pose.
   * @param translation [in]  The target of the translation from the base frame.
   * @param quaternion  [in]  The target of the quaternion related to the base frame.
   * @param jnt_pos     [out] The result of joint position.
   * @return Return true, if everything is OK, or return false.
   */
  virtual bool inverse_kinematics(
      const Eigen::Vector3d& translation, const Eigen::Quaterniond& quaternion,
      Eigen::Vector3d& jnt_pos) override;
  virtual bool inverse_kinematics(
        const Eigen::Vector3d& translation,   Eigen::Vector3d& jnt_pos) override;
  virtual bool inverse_kinematics(
        const Eigen::Quaterniond& quaternion, Eigen::Vector3d& jnt_pos) override;

///! The status of robot-leg of getter
public:
  void eef(Eigen::Vector3d& _xyz, Eigen::Quaterniond& _rpy);
  void eef(Eigen::Vector3d& _xyz);
  void eef(Eigen::Quaterniond& _rpy);

public:
  // virtual void asyncMove(bool& ret_ref);
  virtual void move();

public:
  ///! setter target methods
  void jointTarget           (const JntTarget&);
  void jointTarget           (JntType, JntDataType, double);
  void jointTrajectoryTarget (JntType, const Trajectory1d&);
  ///! The default order is knee, hip and yaw
  void jointTrajectoryTarget (const Trajectory3d&);
  void eefOrientationTarget  (const Eigen::Quaterniond&);
  void eefPositionTarget     (const Eigen::Vector3d&);
  void eefTrajectoryTarget   (const Trajectory3d&);

  ///! getter target methods
  const JntTarget&          jointTarget           ();
  const Trajectory1d&       jointTrajectoryTarget ();
  const Eigen::Quaterniond& eefOrientationTarget  ();
  const Eigen::Vector3d&    eefPositionTarget     ();
  const Trajectory3d&       eefTrajectoryTarget   ();

///! These are the helper methods
protected:
  virtual void followJntTrajectory(JntType, const Trajectory1d) /*= 0*/;
  virtual void followJntTrajectory(const Trajectory3d) /*= 0*/;
  virtual void followEefTrajectory(const Trajectory3d) /*= 0*/;

  virtual void execut(const JntTarget&);
  virtual void execut(const Eigen::Quaterniond&);
  virtual void execut(const Eigen::Vector3d&);

///! The targets for each spaces.
protected:
  Trajectory1d       jnt_traj_target_;
  Trajectory3d       jnts_traj_target_;
  JntTarget          jnt_target_;
  Eigen::Vector3d    eef_xyz_target_;
  Eigen::Quaterniond eef_rpy_target_;
  Trajectory3d       eef_traj_target_;

private:
  enum TargetType {
    INVALID_TARGET = -1,
    JNT_CMD,
    JNT_TRAJ,
    EEF_XYZ,
    EEF_RPY,
    EEF_TRAJ,
    N_TARGET_TYPE
  };

  TargetType curr_target_;
  JntType    curr_target_jnt_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_ROBOT_LEG_H_ */
