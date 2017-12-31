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

Eigen::Vector3d QrBody::leg_base(LegType leg) {
  switch (leg) {
  case LegType::FL: return Eigen::Vector3d( body_length(),  body_width(), 0);
  case LegType::FR: return Eigen::Vector3d( body_length(), -body_width(), 0);
  case LegType::HL: return Eigen::Vector3d(-body_length(),  body_width(), 0);
  case LegType::HR: return Eigen::Vector3d(-body_length(), -body_width(), 0);
  default:
    LOG_ERROR << "ERROR LegType";
    return Eigen::Vector3d(0, 0, 0);
    break;
  }
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::QrBody, Label)
