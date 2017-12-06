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

#include <mutex>

namespace qr_control {

class GaitManager
    : public middleware::internal::ResourceManager<GaitBase> {
  SINGLETON_DECLARE(GaitManager)

public:
  /*!
   * The main function what call the active gait to control robot.
   */
  void tick();

  /*!
   * Change the active gait.
   */
  void activate(const MiiString& gait_name);

  virtual void add(GaitBase*) override;

protected:
  GaitBase*                     running_gait_;
  GaitBase*                     active_gait_;
  MiiMap<MiiString, GaitBase*>  gait_list_by_name_;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_GAIT_MANAGER_H_ */
