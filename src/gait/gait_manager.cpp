/*
 * gait_manager.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#include <gait/gait_manager.h>
#include <system/foundation/cfg_reader.h>

namespace qr_control {

const unsigned int INVALID_GAIT_IDX = -1;

SINGLETON_IMPL_NO_CREATE(GaitManager)

GaitManager::GaitManager(const MiiString& prefix)
  : middleware::internal::ResourceManager<GaitBase>(),
    Label(prefix),
    current_active_gait_idx_(INVALID_GAIT_IDX) {

}

GaitManager::~GaitManager() {
  // Nothing to do here.
}

// TODO
bool GaitManager::init() {
  auto cfg = MiiCfgReader::instance();
  MiiString active_gait;
  cfg->get_value(getLabel(), "active", active_gait);
  return true;
}

} /* namespace qr_control */
