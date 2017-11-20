/*
 * testKDL.cpp
 *
 *  Created on: May 17, 2017
 *      Author: silence
 */

#include <glog/logging.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <qr_control/util.h>

class TestKDL {
private:
  // KDL solvers for the end-effectors.
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_sp_;
  boost::shared_ptr<KDL::ChainIkSolverPos> ik_sp_;
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos>> fk_solvers_;
  std::vector<boost::shared_ptr<KDL::ChainIkSolverPos>> ik_solvers_;
  // Temporary storage for KDL joint angle array.
  KDL::JntArray temp_joint_array_;
  // Temporary storage for KDL joint angle array.
  KDL::JntArray temp_joint_solver_;
  // Temporary storage for KDL tip pose.
  KDL::Frame temp_tip_pose_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  int JNT_NUM_OF_LEG;
  int LEG_NUM;

  bool is_init_;
  int counter;
  const int COUNT;

public:
  TestKDL()
    : JNT_NUM_OF_LEG(3), LEG_NUM(4),
      is_init_(false), counter(0), COUNT(200) {
    temp_joint_array_.resize(3);
    temp_joint_solver_.resize(3);
  }

  virtual ~TestKDL() {
    ;
  }

  bool init() {
    std::string param_name = "jnt_num_of_leg";
    if (!ros::param::get(param_name, JNT_NUM_OF_LEG)) {
      LOG_ERROR << "Controller could not find the parameter: jnt_num_of_leg"
          << ", using the default value: 3";
    }

    param_name = "leg_num";
    if (!ros::param::get(param_name, LEG_NUM)) {
      LOG_ERROR << "Controller could not find the parameter: leg_num"
          << ", using the default value: 4";
    }

    param_name = "robot_description";
    urdf::Model robot_model;
    if (!robot_model.initParam(param_name)) {
      LOG_ERROR << "Failed to parse urdf file";
      return false;
    }
    KDL::Tree temp_kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, temp_kdl_tree)){
      LOG_ERROR << "Could not convert urdf into kdl tree";
      return false;
    }

    // Init fk and ik slover
    int count = 0;
    while(true) {
      param_name = "root_" + std::to_string(count);
      std::string root = "";
      if (!ros::param::get(param_name, root)) {
        break;
      }
      param_name = "tip_" + std::to_string(count);
      std::string tip  = "";
      if (!ros::param::get(param_name, tip)) {
        break;
      }
      // KDL chains for the end-effectors.
      KDL::Chain temp_chain_;
      if (!temp_kdl_tree.getChain(root, tip, temp_chain_)){
        LOG_ERROR << "Controller could not use the chain from '"
          << root << "' to '" << tip << "'";
        return false;
      }
      // Pose solvers.

      fk_sp_.reset(new KDL::ChainFkSolverPos_recursive(temp_chain_));
      fk_solvers_.push_back(fk_sp_);

      ik_sp_.reset(new KDL::ChainIkSolverPos_LMA(temp_chain_));
      ik_solvers_.push_back(ik_sp_);
      ++count;
    }

    sub_ = nh_.subscribe("/joint_states",
        1, &TestKDL::jnt_states_cb, this);
    is_init_ = true;
    return true;
  }

  void jnt_states_cb(const sensor_msgs::JointState::ConstPtr& msg) {
    if (counter++ <= COUNT) return;
    else counter = 0;

    LOG_WARNING << "======================";
    LOG_WARNING << "----------FK----------";
    for (int leg = 0; leg < 4; ++leg) {
      for (int jnt = 0; jnt < 3; ++jnt) {
        LOG_WARNING << 3 * leg + jnt << "/" << msg->position.size();
        temp_joint_array_(jnt) = msg->position[3 * leg + jnt];
      }
      LOG_WARNING << "----------FK1----------";
      fk_solvers_[leg]->JntToCart(temp_joint_array_, temp_tip_pose_);
      LOG_WARNING << "----------FK2----------";
      std::stringstream ss;
      for (int jnt = 0; jnt < 3; jnt++) {
        // end_effector_points_[leg](jnt) = temp_tip_pose_.p(jnt);
        ss << temp_tip_pose_.p(jnt) << "\t";
      }
      LOG_WARNING << ss.str();

      ss.clear();
      ss << "----------IK----------";
      // temp_tip_pose_.p(0) += 0.1;
      // temp_tip_pose_.p(1) += 0.1;
      temp_tip_pose_.p(2) += 0.1;
      ik_solvers_[leg]->CartToJnt(temp_joint_array_, temp_tip_pose_, temp_joint_solver_);
      for (unsigned jnt = 0; jnt < 3; jnt++) {
        ss << temp_joint_solver_(jnt) << "\t";
      }

      LOG_WARNING << ss.str();
    }
    LOG_WARNING << "======================";
  }
};

int main(int argc, char* argv[]) {

  google::InitGoogleLogging("testKDL");
  // google::SetLogDestination(google::GLOG_INFO, "/path/to/log/INFO_");
  // google::LogMessage::Init();
  FLAGS_colorlogtostderr = true;
  google::FlushLogFiles(google::GLOG_INFO);
  google::SetStderrLogging(google::GLOG_WARNING);

  ros::init(argc, argv, "testKDL");
  ros::NodeHandle nh;

  TestKDL test;

  if (test.init()) {
    LOG_INFO << "YES!!!";
  } else {
    LOG_FATAL << "NO!!!";
  }

  ros::spin();

  return 0;
}


