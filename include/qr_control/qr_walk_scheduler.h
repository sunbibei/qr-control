#pragma once

// STL
#include <math.h>
#include <vector>
#include <string>
#include <iostream>

// Eigen
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include <urdf/model.h>
#include <angles/angles.h>

// orocos-kdl
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>

// boost
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/condition.hpp>

// user defined
#include <qr_control/util.h>

namespace qr_control {
using namespace hardware_interface;

const int MAX_LEG_NUM = 8;

enum class CTRL_STATE {
  MOVE_TO_INIT = 0,
  CHOOSE_WHICH_LEG,
  ADJUST_COG,
  STEP_FORWARD,
};

class QrWalkScheduler
    : public controller_interface::Controller<PositionJointInterface> {

public:
  QrWalkScheduler();
  virtual ~QrWalkScheduler();

  bool init(PositionJointInterface*, ros::NodeHandle&) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration&) override;

private:
  std::vector<double>      joint_cmds_;
  std::vector<std::string> joint_names_;
  std::vector<JointHandle> joint_handles_;

  urdf::Model robot_model_;

  int LEG_NUM;
  int JNT_NUM;
  std::vector<int> JNT_NUM_OF_EVERY_LEG;

  Eigen::VectorXd last_X_pos_;
  Eigen::VectorXd init_X_pos_;

  std::vector<KDL::Frame> eef_poses_;
  std::vector<KDL::JntArray> eef_solves_;

  // KDL solvers for the end-effectors.
  std::vector<KDL::Chain> leg_chains_;
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos>> fk_solvers_;
  std::vector<boost::shared_ptr<KDL::ChainIkSolverPos>> ik_solvers_;
  // Temporary storage for KDL joint angle, solve and tip array.
  KDL::JntArray temp_joint_angles_;
  KDL::JntArray temp_joint_of_leg_;
  KDL::JntArray temp_joint_solve_;
  KDL::Frame    temp_tip_pose_;

  // update time control
  ros::Time last_update_time_;
  ros::Duration tmp_duration_;
  ros::Duration step_duration_;
  double UPDATE_FREQ; // s

private:
  void fk(std::vector<KDL::Frame>&);
  int  ik(const std::vector<KDL::Frame>&, std::vector<KDL::JntArray>&);


private:
  int loop_count_;

  CTRL_STATE ctrl_mode_;

  unsigned int leg_order_;
  double init_pos_[12] = {0};
  // Angle Angle_Group = {{0,0,-0.547},{0,0,-0.547},{0,0,0.547},{0,0,0.547}};
  Angle Angle_Group = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  Position Foot_Position_Group = {{BODY_LENGTH, BODY_WIDTH, -L0 - L1 - L2},
                                {BODY_LENGTH, -BODY_WIDTH, -L0 - L1 - L2},
                                {-BODY_LENGTH, BODY_WIDTH, -L0 - L1 - L2},
                                {-BODY_LENGTH, -BODY_WIDTH, -L0 - L1 - L2}};

  Angle_Ptr joint_current_positions_ = &Angle_Group;
  Position_Ptr foots_pos_ = &Foot_Position_Group;
  __Position next_foot_pos_ = {0,0,0};
  __Position Pos_start,Cog_adj;

private:
  // for debug
  void printJointPositions();

  void readJointEncoder(Eigen::VectorXd&);

  void fk();
  void ik();

  void goInitPose();
  void cog_adj();
  __Position cal_formula(__Angle_Leg A,int L,int W);
  __Angle_Leg cal_kinematics(__Position P,int L,int W,int Sgn);
  __Position get_eclipse_pos(__Position Start_point, __Position End_point,int Loop);
  void cog_pos_assign(__Position Adj);
  void swing_control();
  void getNextFoothold();

  void ISM(unsigned int Loop_count);
  int Sgn(double a);
  void vec_assign(const Angle_Ptr&, std::vector<double>&);

  float get_adj_pos(float Adj, int t, int T);

  __Position get_stance_velocity(__Position Adj_vec, unsigned int Loop);
  __Position get_stance_acceration(__Position Adj_vec, unsigned int Loop);
  __Position get_CoG_adj_vec(__Position Next_Foothold, unsigned int Swing_Order);
  __Position calcCrossPoint(const __Position&, const __Position&,
      const __Position&, const __Position&);

  __Position calcTriangleIncentre(__Position A, __Position B, __Position C);
  __Position struct_copy(__Position A);
  __Position struct_assign(double x, double y, double z);
};

/*
class FSM {
public:
  enum _STATE {
    MOVE_TO_INIT = 0,
    CHOOSE_WHICH_LEG,
    ADJUST_COG,
    STEP_FORWARD,
  };

  FSM() : state_(MOVE_TO_INIT) {}
  ~FSM() {}

  FSM& operator++() {
    if (STEP_FORWARD == state_) state_ = CHOOSE_WHICH_LEG;
    else ++state_;

    return *this;
  }
  FSM& operator++(int) {
    FSM tmp = *this;
    ++(*this);
    return tmp;
  }

  FSM& operator--() {
    if (CHOOSE_WHICH_LEG == state_) state_  = STEP_FORWARD;
    else if (MOVE_TO_INIT == state_) state_ = MOVE_TO_INIT;
    else --state_;

    return *this;
  }
  FSM& operator--(int) {
    FSM tmp = *this;
    --(*this);
    return tmp;
  }

  bool operator==(_STATE s) {
    return state_ == s;
  }
  bool operator!=(_STATE s) {
    return state_ != s;
  }

protected:
  FSM& operator+=(int) {return *this;}
  FSM& operator-=(int) {return *this;}

  FSM operator+(int) {return *this;}
  FSM operator-(int) {return *this;}

private:
  _STATE state_;
};
*/

} /* end for namespace qr_control */
