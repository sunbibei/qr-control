/*
 * gait_manager.h
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_GAIT_MANAGER_H_
#define INCLUDE_GAIT_GAIT_MANAGER_H_

#include <system/platform/internal/resource_manager.h>
#include "gait_base.h"

namespace qr_control {

class GaitManager
    : public middleware::internal::ResourceManager<GaitBase>,
      public Label {
  SINGLETON_DECLARE(GaitManager, const MiiString& _l = Label::null)

public:
  bool init();
  bool update();

protected:
  unsigned int           current_active_gait_idx_;
  std::vector<GaitBase*> gait_list_by_name_;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_GAIT_MANAGER_H_ */
