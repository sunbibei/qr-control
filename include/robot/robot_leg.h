/*
 * robot_leg.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_ROBOT_LEG_H_
#define INCLUDE_ROBOT_ROBOT_LEG_H_

#include <robot/math_leg.h>

namespace qr_control {

struct JntTarget {
  JntType    jnt_type;
  JntCmdType jnt_cmd_type;
  double     target;
};

class RobotLeg: public MathLeg {
public:
  RobotLeg();
  virtual ~RobotLeg();

///! The status of robot-leg of getter
public:
  void jointPosition(Eigen::Vector3d& _p);
  void eef(Eigen::Vector3d& _xyz, Eigen::Quaterniond& _rpy);
  void eef(Eigen::Vector3d& _xyz);
  void eef(Eigen::Quaterniond& _rpy);

public:
  // virtual void asyncMove(bool& ret_ref);
  virtual void move();

public:
  ///! setter target methods
  void jointTarget           (const JntTarget&);
  void jointTarget           (JntType, JntCmdType, double);
  void jointTrajectoryTarget (const class Trajectory&);
  void eefOrientationTarget  (const Eigen::Quaterniond&);
  void eefPositionTarget     (const Eigen::Vector3d&);
  void eefTrajectoryTarget   (const class Trajectory&);

  ///! getter target methods
  const JntTarget&          jointTarget           ();
  const class Trajectory&   jointTrajectoryTarget ();
  const Eigen::Quaterniond& eefOrientationTarget  ();
  const Eigen::Vector3d&    eefPositionTarget     ();
  const class Trajectory&   eefTrajectoryTarget   ();

///! These are the helper methods
protected:
  virtual void followJntTrajectory(const class Trajectory*) = 0;
  virtual void followEefTrajectory(const class Trajectory*) = 0;

  virtual void execut(const JntTarget&);
  virtual void execut(const Eigen::Quaterniond&);
  virtual void execut(const Eigen::Vector3d&);

protected:
  class Trajectory*  jnt_traj_target_;
  JntTarget          jnt_target_;
  Eigen::Vector3d    eef_xyz_target_;
  Eigen::Quaterniond eef_rpy_target_;
  class Trajectory*  eef_traj_target_;

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
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_ROBOT_LEG_H_ */
