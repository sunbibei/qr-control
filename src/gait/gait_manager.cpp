/*
 * gait_manager.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#include "gait/gait_manager.h"
#include <foundation/cfg_reader.h>

namespace qr_control {

SINGLETON_IMPL(GaitManager)

GaitManager::GaitManager()
  : middleware::internal::ResourceManager<GaitBase>(),
    running_gait_(nullptr), active_gait_(nullptr) {

}

GaitManager::~GaitManager() {
  // Nothing to do here.
}

/*void GaitManager::add(GaitBase* gait) {
  middleware::internal::ResourceManager<GaitBase>::add(gait);

  gait_list_by_name_.insert(std::make_pair(gait->gaitName(), gait));
}*/

bool GaitManager::init() {
  for (const auto& g : res_list_) {
    gait_list_by_name_.insert(std::make_pair(g->name(), g));
  }

  return true;
}

void GaitManager::tick() {
  if ((!running_gait_) && (!active_gait_)) {
    ///! No gait be running, and no active gait.
    return;
  } else if ((!running_gait_) && (active_gait_)) {
    ///! No gait be running, and an active gait.
    if (active_gait_->starting()) {
      running_gait_ = active_gait_;
    } else {
      LOG_ERROR << "The named " << active_gait_->name()
          << " gait is starting fail.";
      active_gait_->stopping();
      active_gait_ = nullptr;
      return;
    }
  } else if (running_gait_ != active_gait_) {
    ///! The running gait is not the active gait.
    if (running_gait_->canSwitch()) {
      running_gait_->stopping();
      running_gait_ = nullptr;
    } else
      LOG_EVERY_N(WARNING, 10) << "Waiting to switch from "
        << running_gait_->name() << " to " << active_gait_->name();
  } else { // runing_gait_ == active_gait_
    ; // Unreachable code
  }

  ///! prev tick
  running_gait_->prev_tick();

  ///! check state change.
  running_gait_->checkState();
  if (nullptr == running_gait_->state_machine()) {
    LOG_WARNING << "No StateMachine!";
    return;
  }
  ///! call the callbacks
  running_gait_->state_machine()->operator ()();

  ///! post tick
  running_gait_->post_tick();
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

void GaitManager::print() {
  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_WARNING << "NAME\t\tLABEL\t\tADDR";
    for (auto hw : res_list_) {
      LOG_INFO << hw->gait_name_ << "\t" << hw->getLabel() << "\t" << hw;
    }
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_INFO << "The current running gait: " << (running_gait_ ? running_gait_->gait_name_ : "NULL");
    LOG_INFO << "The current active  gait: " << (active_gait_  ? active_gait_->gait_name_  : "NULL");
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }
}

} /* namespace qr_control */
