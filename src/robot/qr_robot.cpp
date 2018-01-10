/*
 * qr_robot.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#include <robot/qr_robot.h>

namespace qr_control {

SINGLETON_IMPL_NO_CREATE(QrRobot)

QrRobot* QrRobot::create_instance(const MiiString& _tag) {
  if (nullptr != instance_) {
    LOG_WARNING << "This method 'create_instance()' is called twice.";
  } else {
    instance_ = new QrRobot(_tag);
  }
  return instance_;
}

QrRobot::QrRobot(const MiiString& _tag)
  : LegRobot(_tag) {
  ;
}

QrRobot::~QrRobot() {
  ; // Nothing to do here.
}

double QrRobot::stability_margin() const {
  // TODO
  return 0.0;
}

} /* namespace qr_control */
