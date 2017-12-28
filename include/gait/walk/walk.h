/*
 * walk.h
 *
 *  Created on: Dec 27, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_WALK_WALK_H_
#define INCLUDE_GAIT_WALK_WALK_H_

#include "gait/gait_base.h"

#include "ultility.h"
#include "math.h"

///! Forward declaration
class TimeControl;

namespace qr_control {

enum WalkState {
  UNKNOWN_WK_STATE = -1,
  WK_WAITING = 0,
  WK_INIT_POSE,
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
  virtual void              prev_tick() override;
  virtual void              post_tick() override;

protected:
  WalkState                  current_state_;
  ///! The state machine corresponds with CreepState for creep
  StateMachine<WalkState>*   state_machine_;
  ///! The interface for body
  class QrBody*         body_iface_;
  ///! The interface for legs
  class QrLeg*          leg_ifaces_[LegType::N_LEGS];
  ///! The commands of legs.
  class LegTarget*      leg_cmds_[LegType::N_LEGS];
  ///! Whether is hang?
  bool            is_hang_walk_;
  ///! The Control tick interval(in ms)
  int64_t         tick_interval_;
  ///! The temporay tick interval(in ms)
  int64_t         sum_interval_;


///! These variable is temporary.
private:
  TimeControl*          timer_;
  bool                  is_send_init_cmds_;
  ///! The internal flag
  unsigned int          internal_order_;
  ///! The current swing leg
  LegType               swing_leg_;

///! These methods are the callback method for WalkState.
private:
  ///! The callback for WK_WAITING
  void waiting();
  ///! The callback for WK_INIT_POSE
  void pose_init();
  ///! The callback for WK_XXX
  void walk();
  ///! The debug callback for WK_HANG
  void hang_walk();

private:
  ///! Choice the next swing leg @next by @curr LegType.
  LegType choice_next_leg(const LegType curr);
  void    next_foot_pt();
  void    move_cog();
  void    swing_leg(const LegType&);
  void    on_ground(const LegType&);
  void    cog_swing(const _Position&, const LegType&);
  _Position    get_CoG_adj_vec(const _Position&, LegType);

/////////////////////////////////////////////////////////////
///////////////////////// OLD CODE //////////////////////////
/////////////////////////////////////////////////////////////
private:
  void forward_kinematics();
  void reverse_kinematics();

  void cog_pos_assign(_Position Adj);

  void command_assign(const Angle&);

  _Position innerTriangle(const _Position &A, const _Position &B, const _Position &C);
  _Position get_stance_velocity(_Position Adj_vec, unsigned int Loop);

protected:
  // Math* math;

  // std_msgs::Float64MultiArray msg;
  // boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> joint_state_publisher_;
private:
  int Loop_Count;

  Position shoulder_ = {{Body_L, Body_W, 0},
      {Body_L, -Body_W, 0},
      {-Body_L, Body_W, 0},
      {-Body_L, -Body_W, 0}};
  Angle jnts_pos_ = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  Position foots_pos_ = { {0, 0, -L0 - L1 - L2},
                          {0, 0, -L0 - L1 - L2},
                          {0, 0, -L0 - L1 - L2},
                          {0, 0, -L0 - L1 - L2}};

  _Position Desired_Foot_Pos = {0,0,0};
  _Position Pos_start,Cog_adj;
  _Position swing_adj_CoG;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_WALK_WALK_H_ */
