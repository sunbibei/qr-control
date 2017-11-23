/*
 * gait_manager.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#include <gait/gait_manager.h>

namespace qr_control {

SINGLETON_IMPL(GaitManager)

GaitManager::GaitManager()
  : middleware::internal::ResourceManager<GaitBase>(),
    running_gait_(nullptr), active_gait_(nullptr) {

}

GaitManager::~GaitManager() {
  // Nothing to do here.
}

bool GaitManager::init() {
  for (auto gait : res_list_) {
    gait_list_by_name_.insert(std::make_pair(gait->gaitName(), gait));
  }

  return true;
}

void GaitManager::tick() {

  if ((!running_gait_) && (!active_gait_)) {
    return;
  } else if ((!running_gait_) && (active_gait_)) {
    running_gait_ = active_gait_;
  } else if (running_gait_ != active_gait_) {
    if (running_gait_->canSwitch())
      running_gait_ = active_gait_;
    else
      LOG_EVERY_N(WARNING, 10) << "Waiting to switch from "
        << running_gait_->gaitName() << " to " << active_gait_->gaitName();
  } else { // runing_gait_ == active_gait_
    ;
  }

  running_gait_->update();
}

void GaitManager::activate(const MiiString& gait_name) {
  auto new_gait = gait_list_by_name_.find(gait_name);
  if (gait_list_by_name_.end() == new_gait) {
    LOG_WARNING << "No gait '" << gait_name << "' register in the gait manager.";
    return;
  }

  LOG_INFO << "Switch the current gait to " << gait_name;
  active_gait_ = new_gait->second;
}

} /* namespace qr_control */
