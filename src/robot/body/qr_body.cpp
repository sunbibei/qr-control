/*
 * qr_body.cpp
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#include <robot/body/qr_body.h>

namespace qr_control {

QrBody::QrBody() {
  // TODO Auto-generated constructor stub

}

QrBody::~QrBody() {
  // TODO Auto-generated destructor stub
}

///! The pose of robot against the world frame
void QrBody::pose(Eigen::Vector3d&, Eigen::Quaterniond&) {
  ;
}

///! The translation of robot against the world frame
void QrBody::translation(EV3&) /*= 0*/ {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
}

///! The rotation of robot against the world frame
void QrBody::rotation(Eigen::Quaterniond&) /*= 0*/ {
  LOG_ERROR << "Call the 'rotation' which has does not complemented.";
}

///! The velocity of robot against the world frame
void QrBody::velocity(EV3& v)  /*= 0*/ {
  LOG_ERROR << "Call the 'velocity' which has does not complemented.";
}

///! The centre of gravity of robot
void QrBody::cog(EV3&)         /*= 0*/ {
  LOG_ERROR << "Call the 'cog' which has does not complemented.";
}

void QrBody::leg_base(LegType leg, Eigen::Vector3d& _xyz) {
  switch (leg) {
  case LegType::FL:
    _xyz << body_length(), body_width(), 0;
    break;
  case LegType::FR:
    _xyz << body_length(), -body_width(), 0;
    break;
  case LegType::HL:
    _xyz << -body_length(), body_width(), 0;
    break;
  case LegType::HR:
    _xyz << -body_length(), -body_width(), 0;
    break;
  default:
    LOG_ERROR << "ERROR LegType";
    break;
  }
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::QrBody, Label)
