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

bool Origin::registerResource(const MiiString& _n, DataTypeFrom _handle) {
  if (data_origin_.end() != data_origin_.find(_n)) {
    LOG_WARNING << "The named resource '" << _n << "' has registered in the resource table."
        << ", now it will be replaced.";
  }

  data_origin_[_n] = _handle;
}

bool Origin::registerCommand(const MiiString& _n, DataTypeTo _handle) {
  if (cmd_origin_.end() != cmd_origin_.find(_n)) {
    LOG_WARNING << "The named command '" << _n << "' has registered in the command table."
        << ", now it will be replaced.";
  }
  
  cmd_origin_[_n] = _handle;
}

} /* namespace qr_control */
