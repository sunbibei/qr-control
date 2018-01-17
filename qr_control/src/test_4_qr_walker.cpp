/*
 * test_4_qr_walker.cpp
 *
 *  Created on: May 17, 2017
 *      Author: silence
 */

#include "test_4_qr_walker.h"

#include <qr_control/util.h>

#include <urdf/model.h>

namespace qr_control {

Test4QrWalker::Test4QrWalker()
  : LEG_NUM(0), JNT_NUM(0),
    UPDATE_FREQ(2) {
  google::InitGoogleLogging("QrWalkScheduler");
  // google::SetLogDestination(google::GLOG_INFO, "/path/to/log/INFO_");
  // google::LogMessage::Init();
  FLAGS_colorlogtostderr = true;
  google::FlushLogFiles(google::GLOG_INFO);
  google::SetStderrLogging(google::GLOG_WARNING);

  joint_handles_.reserve(MAX_LEG_NUM);
  fk_solvers_.reserve(MAX_LEG_NUM);
  ik_solvers_.reserve(MAX_LEG_NUM);
  chains_.reserve(MAX_LEG_NUM);
}

Test4QrWalker::~Test4QrWalker() {
  // TODO Auto-generated destructor stub
}

bool Test4QrWalker::init(
    PositionJointInterface* robot,
    ros::NodeHandle& nh) {
  // get update frequency
  ros::param::get("update_freq", UPDATE_FREQ);

  urdf::Model robot_model;
  if (!robot_model.initParam("robot_description")) {
    LOG_ERROR << "Failed to parse urdf file";
    return false;
  }

  KDL::Tree temp_kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(robot_model, temp_kdl_tree)){
    LOG_ERROR << "Could not convert urdf into kdl tree";
    return false;
  }

  // Init fk and ik slover
  std::string param_name;
  LEG_NUM = 0;
  while(true) {
    param_name = "root_" + std::to_string(LEG_NUM);
    std::string root = "";
    if (!ros::param::get(param_name, root)) {
      break;
    }
    param_name = "tip_" + std::to_string(LEG_NUM);
    std::string tip  = "";
    if (!ros::param::get(param_name, tip)) {
      break;
    }
    // KDL chains for the end-effectors.
    KDL::Chain chain;
    if (!temp_kdl_tree.getChain(root, tip, chain)){
      LOG_ERROR << "Controller could not use the chain from '"
        << root << "' to '" << tip << "'";
      return false;
    }
    chains_.push_back(chain);
    // Pose solvers.
    boost::shared_ptr<KDL::ChainFkSolverPos> fk_sp;
    fk_sp.reset(new KDL::ChainFkSolverPos_recursive(chains_[LEG_NUM]));
    fk_solvers_.push_back(fk_sp);

    boost::shared_ptr<KDL::ChainIkSolverPos> ik_sp;
    ik_sp.reset(new KDL::ChainIkSolverPos_LMA(chains_[LEG_NUM]));
    ik_solvers_.push_back(ik_sp);

    JNT_NUM_OF_EVERY_LEG.push_back(chain.getNrOfJoints());
    ++LEG_NUM;
  }

  /**
   * For debug
  LOG_WARNING << "FK";
  for (const auto& itr : fk_solvers_) {
    LOG_WARNING << itr.use_count() << ", " << itr.get();
  }
  LOG_WARNING << "IK";
  for (const auto& itr : ik_solvers_) {
    LOG_WARNING << itr.use_count() << ", " << itr.get();
  }
   */

  JNT_NUM = 0;
  while(true) {
    std::string joint_name;
    std::string param_name = std::string("joint_" + std::to_string(JNT_NUM));
    if (ros::param::get(param_name.c_str(), joint_name)) {
      joint_handles_.push_back(robot->getHandle(joint_name));
    } else {
      break;
    }
    JNT_NUM++;
  }

  last_X_pos_.resize(JNT_NUM);
  last_X_pos_.fill(0.0);

  end_effector_points_.resize(LEG_NUM);

  step_duration_ = ros::Duration(UPDATE_FREQ);

  return true;
}

void Test4QrWalker::starting(const ros::Time& time) {
  last_update_time_ = time;
}

void Test4QrWalker::stopping(const ros::Time&) {
  ;
}

void Test4QrWalker::update(const ros::Time& time, const ros::Duration& period) {
  tmp_duration_ = time - last_update_time_;
  if (period < (step_duration_ - tmp_duration_)) return;
  else last_update_time_ = time;

  LOG_WARNING << "======================";
  LOG_WARNING << "----------FK----------";
  for (int leg = 0; leg < LEG_NUM; ++leg) {
    if (JNT_NUM_OF_EVERY_LEG[leg] != temp_joint_array_.data.size())
      temp_joint_array_.resize(JNT_NUM_OF_EVERY_LEG[leg]);

    for (int jnt = 0; jnt < JNT_NUM_OF_EVERY_LEG[leg]; ++jnt) {
      temp_joint_array_(jnt) = last_X_pos_(JNT_NUM_OF_EVERY_LEG[leg] * leg + jnt);
    }

    fk_solvers_[leg]->JntToCart(temp_joint_array_, temp_tip_pose_);
    std::stringstream ss;
    for (int coor = 0; coor < 3; ++coor) {
      // end_effector_points_[leg](jnt) = temp_tip_pose_.p(jnt);
      ss << temp_tip_pose_.p(coor) << "\t";
    }
    LOG_WARNING << ss.str();
    ss.str("");

    LOG_WARNING << "----------IK----------";
    if (JNT_NUM_OF_EVERY_LEG[leg] != temp_joint_solver_.data.size())
      temp_joint_solver_.resize(JNT_NUM_OF_EVERY_LEG[leg]);

    temp_joint_solver_.data.setRandom();
    temp_joint_solver_.data *= M_PI;
    fk_solvers_[leg]->JntToCart(temp_joint_solver_, temp_tip_pose_);
    for (int coor = 0; coor < 3; ++coor) {
      // end_effector_points_[leg](jnt) = temp_tip_pose_.p(jnt);
      ss << temp_tip_pose_.p(coor) << "\t";
    }
    ss << "\nANSW:";
    for (int jnt = 0; jnt < JNT_NUM_OF_EVERY_LEG[leg]; jnt++) {
      ss << "\t" << temp_joint_solver_(jnt);
    }

    ik_solvers_[leg]->CartToJnt(temp_joint_array_, temp_tip_pose_, temp_joint_solver_);
    //ss << "\nINIT:";
    //for (int jnt = 0; jnt < JNT_NUM_OF_EVERY_LEG[leg]; jnt++) {
    //  ss << "\t" << temp_joint_array_(jnt);
    //}

    ss << "\nI  K:";
    for (int jnt = 0; jnt < JNT_NUM_OF_EVERY_LEG[leg]; jnt++) {
      ss << "\t" << temp_joint_solver_(jnt);
    }

    LOG_WARNING << ss.str();
    ss.str("");
  }
  LOG_WARNING << "======================";
  /*
  LOG_WARNING << "----------IK----------";
  for (unsigned leg = 0; leg < 4; ++leg) {
    for (unsigned jnt = 0; jnt < 3; ++jnt) {
      temp_joint_array_(jnt) = last_X_pos_(3 * leg + jnt);
    }

    KDL::Vector V(end_effector_points_[leg](0),
        end_effector_points_[leg](1), end_effector_points_[leg](2));
    KDL::Rotation R = KDL::Rotation::Quaternion(0, 0, 0, 1);
    KDL::Frame goal(R, V);

    ik_solvers_[leg]->CartToJnt(temp_joint_array_, goal, temp_joint_solver_);
    std::stringstream ss;
    for (unsigned jnt = 0; jnt < 3; jnt++) {
      ss << temp_joint_solver_(jnt) << "\t";
    }
    LOG_WARNING << ss.str();
  }
  */


}

} /* namespace qr_control */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    qr_control::Test4QrWalker,
    controller_interface::ControllerBase)
