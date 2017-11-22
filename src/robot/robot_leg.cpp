/*
 * robot_leg.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#include "robot/robot_leg.h"
#include "robot/origin/origin.h"

#include <system/foundation/cfg_reader.h>

namespace qr_control {

RobotLeg::RobotLeg()
  : Label("robot-leg"),jnt_type_(JntType::UNKNOWN_JNT),
    foot_force_(nullptr), jnt_pos_(nullptr), jnt_cmd_(nullptr) {
  ;
}

bool RobotLeg::init() {
  auto cfg    = MiiCfgReader::instance();
  auto origin = Origin::instance();
  cfg->get_value_fatal(getLabel(), "jnt", jnt_type_);

  ///! The 'origin' attribute should be has two elements.
  MiiVector<MiiString> tmp_vec;
  cfg->get_value_fatal(getLabel(), "origin", tmp_vec);
  if (3 != tmp_vec.size()) {
    // Two elements, [state name, command name, foot force]
    LOG_FATAL << "The error attribute of RobotLeg: " << getLabel();
  }

  jnt_pos_    = origin->resource<const Eigen::VectorXd*>(tmp_vec[0]);
  jnt_cmd_    = origin->command<Eigen::VectorXd*>(tmp_vec[1]);
  foot_force_ = origin->resource<const short*>(tmp_vec[2]);
  return true;
}

RobotLeg::~RobotLeg() {
  ; // Nothing to do here.
}

} /* namespace qr_control */

/*#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::RobotLeg, Label)*/
