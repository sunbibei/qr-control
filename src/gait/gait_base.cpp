/*
 * gait_base.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#include "gait/gait_base.h"
#include "gait/gait_manager.h"
#include <foundation/cfg_reader.h>


namespace qr_control {
StateMachineBase::StateMachineBase()  { }

StateMachineBase::~StateMachineBase() { }

GaitBase::GaitBase(const MiiString& _l)
  : Label(_l) {
  // Register gait class pointer into manager.
  GaitManager::instance()->add(this);
}

bool GaitBase::init() {
  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "name", gait_name_);
  return true;
}

GaitBase::~GaitBase() {
  // Nothing to do here.
}

void GaitBase::update() {
  checkState();
  if (nullptr == state_machine()) {
    LOG_WARNING << "No StateMachine!";
    return;
  }
  state_machine()->operator ()();
}

bool GaitBase::canSwitch() {
  LOG_ERROR << "Call the base method 'canSwitch'";
  return true;
}

void GaitBase::checkState() {
  LOG_ERROR << "Call the base method 'checkState'";
}

StateMachineBase* GaitBase::state_machine() {
  LOG_ERROR << "Call the base method 'checkState'";
  return nullptr;
}

} /* namespace qr_control */
