/*
 * walk.h
 *
 *  Created on: Dec 27, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_WALK_WALK_H_
#define INCLUDE_GAIT_WALK_WALK_H_

#include "gait/gait_base.h"
#include "robot/leg/qr_leg.h"

#include "ultility.h"
#include "foot_contact.h"
#include "swing.h"
#include "math.h"
#include "gesture_control.h"

#include <repository/resource/joint.h>
#include <repository/resource/imu_sensor.h>
#include <repository/resource/force_sensor.h>
#include <toolbox/timer.h>

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

  ///! The interface for legs
  QrLeg*          leg_ifaces_[LegType::N_LEGS];
  ///! The commands of legs.
  LegTarget       leg_cmds_[LegType::N_LEGS];
  ///! Whether is hang?
  bool            is_hang_walk_;

///! These variable is temporary.
private:
  middleware::Timer*    timer_;
  bool                  is_send_init_cmds_;
  ///! The internal flag
  unsigned int          internal_order_;
  ///! The current swing leg
  LegType               swing_leg_;
  ///! The leg whether stand flag
  bool                  support_legs_[LegType::N_LEGS];

  ///! These variables need to delete!
  std::vector<middleware::ForceSensor*>       td_handles_;
  middleware::ImuSensor*                      imu_handle_;

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

private:
  void update_shoulder_pos(float pitch,float yaw,float roll);
  void forward_kinematics();
  void reverse_kinematics();

  void cog_adj1();
  void flow_control1(int timeorder);
  void cog_pos_assign(_Position Adj);
  void cog_swing_assign1(_Position Adj, int legId);

  void swing_control1();
  void hardware_delay_test();
  void assign_next_foot1();
  void on_ground_control1(int legId);

  void command_assign(Angle_Ptr Angle);
  void print_command();

  bool stand_stable(std::vector<bool> IsContact);
  bool contact_keep(std::vector<bool> IsContact);
  // void sensor_height();
  void posture_keep(std::vector<bool> IsContact);

  void forward_control(float Kp,float Kv,float Kd);
  float single_forward_control(float d_pos, float d_vel, float Kp, float Kv, float Kd, float f_pos, float f_vel);

  _Position innerTriangle(const _Position &A, const _Position &B, const _Position &C);
  _Position get_stance_velocity(_Position Adj_vec, unsigned int Loop);

  _Position get_CoG_adj_vec1(const _Position &Next_Foothold, unsigned int Swing_Order);

protected:
  float test_tmp;
  int n_joints_;
  int foot_sensor_const;
  FootContact* foot_contact1;
  Swing* swing;
  Math* math;
  Gesture* gesture;

  Quartic Height;

  // std_msgs::Float64MultiArray msg;
  // boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> joint_state_publisher_;
private:
  int Loop_Count;
  int Transfer_Point = 0;
  int Switch = 0;
  int count_loop = 0;
  bool Leg_On_Ground = 0;

  unsigned int Leg_Order1 = 2;
  double Init_Pos[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  // Angle Angle_Group = {{0,0,-0.547},{0,0,-0.547},{0,0,0.547},{0,0,0.547}};
  Angle Angle_Group = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

  Position Foot_Position_Group = {{0, 0, -L0 - L1 - L2},
  {0, 0, -L0 - L1 - L2},
  {0, 0, -L0 - L1 - L2},
  {0, 0, -L0 - L1 - L2}
  };
  bool Isstand_stable;
  bool All_on_ground;
  // std::vector<bool> IsContact;
  std::vector<float> height;
  std::vector<bool> SupportLeg;

  // std::vector<Commands> commands;

  Angle_Ptr Angle_ptr = &Angle_Group;
  Position_Ptr Pos_ptr = &Foot_Position_Group;
  _Position Desired_Foot_Pos = {0,0,0};
  _Position Pos_start,Cog_adj;
  _Position swing_adj_CoG;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_WALK_WALK_H_ */
