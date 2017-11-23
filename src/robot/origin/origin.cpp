/*
 * origin.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#include <robot/origin/origin.h>

namespace qr_control {

SINGLETON_IMPL_NO_CREATE(Origin)

Origin* Origin::create_instance() {
  if (nullptr != instance_) {
    LOG_WARNING << "This method 'create_instance()' is called twice.";
  } else {
    instance_ = make_instance();
  }

  return instance_;
}

Origin::Origin()
  : Label("origin") {
}

Origin::~Origin() {
  ;
}

} /* namespace qr_control */
