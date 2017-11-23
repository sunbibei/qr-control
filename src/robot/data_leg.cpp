/*
 * data_leg.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#include <robot/data_leg.h>
#include <system/foundation/cfg_reader.h>

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

} /* namespace qr_control */
