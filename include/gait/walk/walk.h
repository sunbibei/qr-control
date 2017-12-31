/*
 * walk.h
 *
 *  Created on: Dec 27, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_WALK_WALK_H_
#define INCLUDE_GAIT_WALK_WALK_H_

#include "gait/gait_base.h"
#include "adt/trajectory.h"

#include <Eigen/Dense>

// #define PUB_ROS_TOPIC
#ifdef PUB_ROS_TOPIC
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#endif

///! Forward declaration
class TimeControl;
// class Trajectory3d;

namespace qr_control {

enum WalkState {
  UNKNOWN_WK_STATE = -1,
  WK_WAITING = 0,
  WK_INIT_POSE,
  WK_MOVE_COG,
  WK_SWING,
  WK_HANG,
  N_WK_STATE,
};

class Walk: public GaitBase {
public:
  Walk();
  virtual bool init() override;

  virtual ~Walk();

///! These methods are inherited from super class.
public:
  ///! The flow of state convert
  virtual void              checkState()    override;
  virtual StateMachineBase* state_machine() override;

  ///! call it before the state callback
  // virtual void              prev_tick() override;
  ///! call it after the state callback
  virtual void              post_tick() override;

  ///! starting the gait
  virtual bool              starting()  override;
  ///! stopping the gait
  virtual void              stopping()  override;

protected:
  WalkState                  current_state_;
  ///! The state machine corresponds with CreepState for creep
  StateMachine<WalkState>*   state_machine_;

///! These methods are the callback method for WalkState.
private:
  ///! The callback for WK_WAITING
  void waiting();
  ///! The callback for WK_INIT_POSE
  void pose_init();
  ///! The callback for WK_MOVE_COG
  void move_cog();
  ///! The callback for WK_SWING
  void swing_leg();
  ///! The callback for WK_XXX
  void walk();
  ///! The debug callback for WK_HANG
  void hang_walk();

///! The helper for move COG and swing leg.
private:
  /*!
   * @brief Update the next foot point for the swing leg.
   *        The default next foot point is {STEP, XX, -BH},
   *        namely the swing leg move forward STEP cm. The
   *        trajectory is quadratic curve.
   */
  void prog_eef_traj();
  /*!
   * @brief Programming the trajectory of COG, and convert to the trajectory
   *        of end-effector. This trajectory is linearity curve.
   */
  void prog_cog_traj();
private:
  /*!
   * @brief Choice the next swing leg @next by @curr LegType.
   *        The flow of swing leg is designed by this method.
   *        The order as follow:
   *        HL -> FL -> HR -> FR
   *
   *        **NOTE**: This method ONLY be called after the moving COG.
   */
  LegType next_leg(const LegType);

  /*!
   * @brief It get a reference of next COG coordination when given the current
   *        swing leg.
   */
  Eigen::Vector2d prog_next_cog(LegType);
protected:
  ///! The interface for body
  class QrBody*         body_iface_;
  ///! The interface for legs
  class QrLeg*          leg_ifaces_[LegType::N_LEGS];
  ///! The commands of legs.
  class LegTarget*      leg_cmds_[LegType::N_LEGS];
  ///! Variables about gait control
  class WalkCoeff* coeff_;
  ///! The time control
  TimeControl*     timer_;
  ///! The trajectory for swing leg
  Trajectory3d*    eef_traj_;
  ///! The trajectory for moving COG
  Trajectory3d*    eef2cog_traj_[LegType::N_LEGS];

  ///! The current swing leg
  LegType          swing_leg_;
  ///! The Control tick interval for send command(in ms)
  int64_t          post_tick_interval_;

#ifdef PUB_ROS_TOPIC
  boost::scoped_ptr<ros::NodeHandle> nh_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<
    std_msgs::Float64MultiArray>> cmd_pub_;
#endif

///! These variables and methods are temporary.
private:
  // Eigen::Vector2d cog_proj1();
  ///! The last position of swing leg
  Eigen::Vector3d last_foot_pos1_;
  ///! The target position of swing leg
  Eigen::Vector3d next_foot_pos1_;
  ///! Whether is hang?
  // bool            is_hang_walk_;
//  Eigen::Vector2d stance_velocity(const Eigen::Vector2d&, int);
//  Eigen::Vector2d stance_velocity(const Eigen::Vector2d&, int64_t);
  // Eigen::Vector2d inner_triangle(const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);

};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_WALK_WALK_H_ */
