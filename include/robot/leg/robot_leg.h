/*
 * robot_leg.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_LEG_ROBOT_LEG_H_
#define INCLUDE_ROBOT_LEG_ROBOT_LEG_H_

#include <adt/trajectory.h>
#include <robot/leg/math_leg.h>

namespace qr_control {

struct JntTarget {
  JntType     jnt_type;
  JntDataType jnt_cmd_type;
  double      target;
};

class RobotLeg: public MathLeg {
public:
  RobotLeg();
  virtual ~RobotLeg();

///! The status of robot-leg of getter
public:
  void eefOriPos(EVX& _rpy, EVX& _xyz);
 
public:
  /*!
   * @brief This method will be return immediately, the process of control is
   *        running in a detach thread.
   * @param remainder The remainder time in ms.
   */
  // virtual void asyncMove(size_t& ret_ref);
  /*!
   * @brief This method will be block until the process of control
   *        has completed.
   */
  virtual void move();

public:
  ///! setter target methods
  void jointTarget           (const JntTarget&);
  void jointTarget           (JntType, JntDataType, double);
  void jointTrajectoryTarget (JntType, const Trajectory1d&);
  ///! The default order is knee, hip and yaw
  void jointTrajectoryTarget (const Trajectory3d&);
  void eefOrientationTarget  (const Eigen::Quaterniond&);
  void eefPositionTarget     (const EV3&);
  void eefTrajectoryTarget   (const Trajectory3d&);

  ///! getter target methods
  const JntTarget&          jointTarget           ();
  const Trajectory1d&       jointTrajectoryTarget ();
  const Eigen::Quaterniond& eefOrientationTarget  ();
  const EV3&    eefPositionTarget     ();
  const Trajectory3d&       eefTrajectoryTarget   ();

///! These are the helper methods
protected:
  virtual void followJntTrajectory(JntType, const Trajectory1d) = 0;
  virtual void followJntTrajectory(const Trajectory3d) = 0;
  virtual void followEefTrajectory(const Trajectory3d) = 0;

  virtual void execut(const JntTarget&);
  virtual void execut(const EVX&);

///! The targets for each spaces.
protected:
  Trajectory1d       jnt_traj_target_;
  Trajectory3d       jnts_traj_target_;
  JntTarget          jnt_target_;
  EV3                eef_xyz_target_;
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

#endif /* INCLUDE_ROBOT_LEG_ROBOT_LEG_H_ */
