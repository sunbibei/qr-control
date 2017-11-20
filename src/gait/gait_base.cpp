/*
 * gait_base.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#include <gait/gait_base.h>
#include <gait/gait_manager.h>
#include <system/foundation/cfg_reader.h>


namespace qr_control {

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

} /* namespace qr_control */
