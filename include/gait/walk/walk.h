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
  WK_INIT = 0,
  WK_SWING,
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
  WalkState                 currentState()  const;

protected:
  WalkState                  current_state_;
  ///! The state machine corresponds with CreepState for creep
  StateMachine<WalkState>*   state_machine_;

private:
  middleware::Timer*         timer_;

private:
  void update();

  void __initAllofData();

  void state_transfer();
  void update_shoulder_pos(float pitch,float yaw,float roll);
  void forward_kinematics();
  void reverse_kinematics();
  void pose_init();
  void cog_adj();
  void flow_control(int timeorder);
  void cog_pos_assign(_Position Adj);
  void cog_swing_assign(_Position Adj, int legId);

  void swing_control();
  void hardware_delay_test();
  void assign_next_foot();
  void on_ground_control(int legId);
  void cog_adj_backward();
  void flow_control_backward(int timeorder);
  void assign_next_foot_backward();
  void assign_next_foot_turn();
  void flow_control_turn(int timeorder);
  void cog_adj_turn();
  void command_assign(Angle_Ptr Angle);
  void command_init();
  void print_command();

  bool stand_stable(std::vector<bool> IsContact);
  bool contact_keep(std::vector<bool> IsContact);
  void sensor_height();
  void posture_keep(std::vector<bool> IsContact);

  void forward_control(float Kp,float Kv,float Kd);
  float single_forward_control(float d_pos, float d_vel, float Kp, float Kv, float Kd, float f_pos, float f_vel);

  _Position innerTriangle(const _Position &A, const _Position &B, const _Position &C);
  _Position get_stance_velocity(_Position Adj_vec, unsigned int Loop);
  float get_stance_velocity(float adj, unsigned int Loop);

  _Position get_CoG_adj_vec(const _Position &Next_Foothold, unsigned int Swing_Order);

protected:
  float test_tmp;
  int n_joints_;
  int foot_sensor_const;
  FootContact* foot_contact;
  Swing* swing;
  Math* math;
  Gesture* gesture;
  bool HangUpWalk;

  Quartic Height;

  std::vector< std::string > joint_names_;
  // std_msgs::Float64MultiArray msg;
  // boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> joint_state_publisher_;
private:
  int Loop_Count;
  int Transfer_Point = 0;
  int Switch = 0;
  int count_loop = 0;
  bool Leg_On_Ground = 0;
  unsigned int Time_Order = 0;
  unsigned int Leg_Order = 2;
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

  std::vector<Commands> commands;
  Angle_Ptr Angle_ptr = &Angle_Group;
  Position_Ptr Pos_ptr = &Foot_Position_Group;
  _Position Desired_Foot_Pos = {0,0,0};
  _Position Pos_start,Cog_adj;
  _Position swing_adj_CoG;

  std::vector<middleware::Joint*>             joint_handles_;
  std::vector<middleware::ForceSensor*>       td_handles_;
  middleware::ImuSensor*                      imu_handle_;

  const double*                  imu_ang_vel_;
  const double*                  imu_lin_acc_;
  const double*                  imu_quat_;

  std::vector<const double*>     jnt_poss_;
  std::vector<const double*>     jnt_vels_;
  std::vector<const double*>     jnt_tors_;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_WALK_WALK_H_ */
