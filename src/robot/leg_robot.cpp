/*
 * leg_robot.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#include <robot/leg_robot.h>

#include <foundation/cfg_reader.h>

namespace qr_control {

LegRobot::LegRobot(const MiiString& _tag)
  : body_iface_(nullptr) {
  for (auto& l : leg_ifaces_)
    l = nullptr;
  auto cfg = MiiCfgReader::instance();
  MiiVector<MiiString> labels;
  cfg->get_value_fatal(_tag, "legs", labels);
  for (const auto& l : labels) {
    auto iface = Label::getHardwareByName<RobotLeg>(l);
    if (nullptr == iface) {
      LOG_ERROR << "Could not find the interface of leg labeled '"
          << l << "'.";
      continue;
    }

    leg_ifaces_[iface->leg_type()] = iface;
  }

  MiiString label;
  cfg->get_value_fatal(_tag, "body", label);
  auto iface = Label::getHardwareByName<RobotBody>(label);
  if (nullptr == iface) {
    LOG_ERROR << "Could not find the interface of leg labeled '"
        << label << "'.";
    return;
  }

  body_iface_ = iface;
}

LegRobot::~LegRobot() {
	; // Nothing to do here.
}

/*!
 * @brief The interface for robot leg.
 */
RobotLeg* LegRobot::robot_leg(LegType _l) {
  return ((LegType::UNKNOWN_LEG == _l) || (LegType::N_LEGS == _l)) ? nullptr : leg_ifaces_[_l];
}

/*!
 * @brief The interface for robot body,
 */
RobotBody* LegRobot::robot_body() {
  return body_iface_;
}

} /* namespace qr_control */
