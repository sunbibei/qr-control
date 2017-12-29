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
  virtual void              checkState()    override;
  virtual StateMachineBase* state_machine() override;
  // virtual void              prev_tick() override;
  virtual void              post_tick() override;
  virtual bool              starting()  override;
  // virtual void              stopping()  override;

protected:
  WalkState                  current_state_;
  ///! The state machine corresponds with CreepState for creep
  StateMachine<WalkState>*   state_machine_;
  ///! The interface for body
  class QrBody*         body_iface_;
  ///! The interface for legs
  class QrLeg*          leg_ifaces_[LegType::N_LEGS];
  ///! The commands of legs. TODO Unused
  class LegTarget*      leg_cmds_[LegType::N_LEGS];
  ///! Whether is hang?
  bool            is_hang_walk_;
  ///! The Control tick interval(in ms)
  int64_t         tick_interval_;
  ///! The temporary tick interval(in ms)
  int64_t         sum_interval_;
  ///! The position of each foot
  Eigen::Vector3d foots_pos_[LegType::N_LEGS];
  ///! The position of joints each leg
  Eigen::VectorXd jnts_pos_cmd_ [LegType::N_LEGS];
  ///! The last position of swing leg
  Eigen::Vector3d last_foot_pos_;
  ///! The target position of swing leg
  Eigen::Vector3d next_foot_pos_;
  ///! Variables about gait control
  class WalkCoeff* coeff_;
  ///! The time control
  TimeControl*     timer_;
  ///! The current swing leg
  LegType          swing_leg_;
  ///! The trajectory for end-effector
  Trajectory3d*    eef_traj_;
  ///! The trajectory for joints
  Trajectory3d*    jnt_traj_;

///! These variable is temporary.
private:
  bool                  is_send_init_cmds_;
  ///! The cog adjust vector
  Eigen::Vector2d       delta_cog_;
  Eigen::Vector2d       swing_delta_cog_;

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

  void reverse_kinematics();
  void forward_kinematics();

#ifdef PUB_ROS_TOPIC
  boost::scoped_ptr<ros::NodeHandle> nh_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<
    std_msgs::Float64MultiArray>> cmd_pub_;
#endif

///! The helper for move COG.
private:
  Eigen::Vector2d delta_cog(LegType);
  Eigen::Vector2d stance_velocity(const Eigen::Vector2d&, int);

///! The helper for swing leg.
private:
  void eef_trajectory();

private:
  /*!
   * @brief Choice the next swing leg @next by @curr LegType.
   *        The flow of swing leg is designed by this method.
   *        The order as follow:
   *        HL -> FL -> HR -> FR
   */
  LegType next_leg(const LegType);
  /*!
   * @brief Update the next foot point for the swing leg.
   *        The default next foot point is {STEP, XX, -BH},
   *        namely the swing leg move forward STEP cm.
   */
  void    next_foot_pt();

  Eigen::Vector2d inner_triangle(const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);
/////////////////////////////////////////////////////////////
///////////////////////// OLD CODE //////////////////////////
/////////////////////////////////////////////////////////////
private:
  int Loop_Count;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_WALK_WALK_H_ */
