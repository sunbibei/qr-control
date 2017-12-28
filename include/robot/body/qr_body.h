/*
 * qr_body.h
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_BODY_QR_BODY_H_
#define INCLUDE_ROBOT_BODY_QR_BODY_H_

#include "math_body.h"

namespace qr_control {

class QrBody: public MathBody {
public:
  QrBody();
  virtual ~QrBody();

///! inherit from MathBody
public:
  ///! The pose of robot against the world frame
  virtual void pose(Eigen::Vector3d&, Eigen::Quaterniond&) /*= 0*/;
  ///! The translation of robot against the world frame
  virtual void translation(EV3&) override /*= 0*/;
  ///! The rotation of robot against the world frame
  virtual void rotation(Eigen::Quaterniond&) override /*= 0*/;
  ///! The velocity of robot against the world frame
  virtual void velocity(EV3& v)  override /*= 0*/;
  ///! The centre of gravity of robot
  virtual void cog(EV3&)         override /*= 0*/;

  virtual void leg_base(LegType, Eigen::Vector3d&) override;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_BODY_QR_BODY_H_ */
