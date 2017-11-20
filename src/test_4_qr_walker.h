/*
 * test_4_qr_walker.h
 *
 *  Created on: May 17, 2017
 *      Author: silence
 */

#ifndef TEST_4_QR_WALKER_H_
#define TEST_4_QR_WALKER_H_

#include <ros/ros.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace qr_control {
using namespace hardware_interface;

const int MAX_LEG_NUM = 8;

class Test4QrWalker
    :public controller_interface::Controller<PositionJointInterface> {

public:
  Test4QrWalker();
  virtual ~Test4QrWalker();

  bool init(PositionJointInterface* robot, ros::NodeHandle& nh);
  void starting(const ros::Time&);
  void stopping(const ros::Time&);
  void update(const ros::Time&, const ros::Duration&);

private:
  std::vector<JointHandle> joint_handles_;
  // KDL solvers for the end-effectors.
  std::vector<KDL::Chain> chains_;
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos>> fk_solvers_;
  std::vector<boost::shared_ptr<KDL::ChainIkSolverPos>> ik_solvers_;
  // Temporary storage for KDL joint angle array.
  KDL::JntArray temp_joint_array_;
  KDL::JntArray temp_joint_solver_;
  // Temporary storage for KDL tip pose.
  KDL::Frame temp_tip_pose_;

  Eigen::VectorXd last_X_pos_;
  std::vector<Eigen::Vector3d> end_effector_points_;

  int LEG_NUM;
  int JNT_NUM;
  std::vector<int> JNT_NUM_OF_EVERY_LEG;

  // update time control
  ros::Time last_update_time_;
  ros::Duration tmp_duration_;
  ros::Duration step_duration_;
  double UPDATE_FREQ; // s
};

} /* namespace qr_control */

#endif /* TEST_4_QR_WALKER_H_ */
