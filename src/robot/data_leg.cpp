/*
 * data_leg.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#include <robot/data_leg.h>
#include <foundation/cfg_reader.h>

namespace qr_control {

DataLeg::DataLeg()
  : Label("robot-leg"), jnt_type_(JntType::UNKNOWN_JNT),
    foot_force_(nullptr), jnt_pos_(nullptr) {
  for (auto& c : jnt_cmd_)
    c = nullptr;
}

bool DataLeg::init() {
  auto cfg    = MiiCfgReader::instance();
  auto origin = Origin::instance();
  cfg->get_value_fatal(getLabel(), "jnt", jnt_type_);

  MiiString tmp_str;
  cfg->get_value_fatal(getLabel(), "position", tmp_str);
  jnt_pos_    = origin->resource<const Eigen::VectorXd*>(tmp_str);

  cfg->get_value_fatal(getLabel(), "tdlo", tmp_str);
  foot_force_    = origin->resource<const short*>(tmp_str);

  ///! The 'command' attribute should be has three elements.
  MiiVector<MiiString> tmp_vec;
  cfg->get_value_fatal(getLabel(), "command", tmp_vec);
  for (size_t i = 0; i < tmp_vec.size() && i < 3; ++i) {
    jnt_cmd_[i] = origin->command<Eigen::VectorXd*>(tmp_vec[i]);
  }

  return true;
}

DataLeg::~DataLeg() {
  ; // Nothing to do here
}

inline JntType DataLeg::joint_type() { return jnt_type_; }

inline short         DataLeg::foot_force()               { return *foot_force_; }
inline const short&  DataLeg::foot_force_const_ref()     { return *foot_force_; }
inline const short*  DataLeg::foot_force_const_pointer() { return foot_force_;  }

inline Eigen::VectorXd        DataLeg::joint_position()               { return *jnt_pos_; }
inline const Eigen::VectorXd& DataLeg::joint_position_const_ref()     { return *jnt_pos_; }
inline const Eigen::VectorXd* DataLeg::joint_position_const_pointer() { return jnt_pos_;  }

// Only get the last command.
inline Eigen::VectorXd        DataLeg::joint_command(JntCmdType _t)         { return *(jnt_cmd_[_t]); }
inline Eigen::VectorXd&       DataLeg::joint_command_ref(JntCmdType _t)     { return *(jnt_cmd_[_t]); }
inline Eigen::VectorXd*       DataLeg::joint_command_pointer(JntCmdType _t) { return jnt_cmd_[_t];    }

} /* namespace qr_control */
