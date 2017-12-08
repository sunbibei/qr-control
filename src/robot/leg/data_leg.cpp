/*
 * data_leg.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#include <foundation/cfg_reader.h>
#include <robot/leg/data_leg.h>
#include "repository/registry.h"

namespace qr_control {

DataLeg::DataLeg()
  : Label("robot-leg"), leg_type_(LegType::UNKNOWN_LEG),
    foot_force_(nullptr), jnt_cmd_(nullptr) {
  for (auto& c : jnt_pos_)
    c = nullptr;
}

bool DataLeg::init() {
  auto cfg    = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "leg", leg_type_);

  MiiString tmp_str;
  cfg->get_value_fatal(getLabel(), "command", tmp_str);
  jnt_cmd_ = GET_COMMAND(tmp_str, Eigen::VectorXd*);

  cfg->get_value_fatal(getLabel(), "tdlo", tmp_str);
  foot_force_  = GET_RESOURCE(tmp_str, const double*);

  cfg->get_value_fatal(getLabel(), "pos", tmp_str);
  jnt_pos_[JntDataType::POS] = GET_RESOURCE(tmp_str, const Eigen::VectorXd*);

  cfg->get_value_fatal(getLabel(), "vel", tmp_str);
  jnt_pos_[JntDataType::VEL] = GET_RESOURCE(tmp_str, const Eigen::VectorXd*);

  cfg->get_value_fatal(getLabel(), "tor", tmp_str);
  jnt_pos_[JntDataType::TOR] = GET_RESOURCE(tmp_str, const Eigen::VectorXd*);

  if (_DEBUG_INFO_FLAG) {
    LOG_INFO << "get command (" << leg_type_ << "): " << jnt_cmd_;
    LOG_INFO << "get resource(JntDataType::TD /" << leg_type_ << "): " << foot_force_;
    LOG_INFO << "get resource(JntDataType::POS/" << leg_type_ << "): " << jnt_pos_[JntDataType::POS];
    LOG_INFO << "get resource(JntDataType::VEL/" << leg_type_ << "): " << jnt_pos_[JntDataType::VEL];
    LOG_INFO << "get resource(JntDataType::TOR/" << leg_type_ << "): " << jnt_pos_[JntDataType::TOR];
  }
  return true;
}

DataLeg::~DataLeg() {
  ; // Nothing to do here
}

LegType DataLeg::leg_type() { return leg_type_; }

double         DataLeg::foot_force()               { return *foot_force_; }
const double&  DataLeg::foot_force_const_ref()     { return *foot_force_; }
const double*  DataLeg::foot_force_const_pointer() { return foot_force_;  }

Eigen::VectorXd        DataLeg::joint_position()               { return *jnt_pos_[JntDataType::POS]; }
const Eigen::VectorXd& DataLeg::joint_position_const_ref()     { return *jnt_pos_[JntDataType::POS]; }
const Eigen::VectorXd* DataLeg::joint_position_const_pointer() { return jnt_pos_[JntDataType::POS];  }

Eigen::VectorXd        DataLeg::joint_velocity()               { return *jnt_pos_[JntDataType::VEL]; }
const Eigen::VectorXd& DataLeg::joint_velocity_const_ref()     { return *jnt_pos_[JntDataType::VEL]; }
const Eigen::VectorXd* DataLeg::joint_velocity_const_pointer() { return jnt_pos_[JntDataType::VEL];  }

Eigen::VectorXd        DataLeg::joint_torque()               { return *jnt_pos_[JntDataType::TOR]; }
const Eigen::VectorXd& DataLeg::joint_torque_const_ref()     { return *jnt_pos_[JntDataType::TOR]; }
const Eigen::VectorXd* DataLeg::joint_torque_const_pointer() { return jnt_pos_[JntDataType::TOR];  }

// Only get the last command.
Eigen::VectorXd        DataLeg::joint_command()         { return *jnt_cmd_; }
Eigen::VectorXd&       DataLeg::joint_command_ref()     { return *jnt_cmd_; }
Eigen::VectorXd*       DataLeg::joint_command_pointer() { return jnt_cmd_;  }

} /* namespace qr_control */
