/*
 * math_body.h
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_BODY_MATH_BODY_H_
#define INCLUDE_ROBOT_BODY_MATH_BODY_H_

#include <robot/body/data_body.h>

namespace qr_control {

class MathBody: public DataBody {
///! These are the
public:
  ///! The pose of robot against the world frame
  virtual void pose(Eigen::Vector3d&, Eigen::Quaterniond&) = 0;
  ///! The translation of robot against the world frame
  virtual void translation(Eigen::Vector3d&) = 0;
  ///! The rotation of robot against the world frame
  virtual void rotation(Eigen::Quaterniond&) = 0;
  ///! The velocity of robot against the world frame
  virtual void velocity(Eigen::Vector3d& v)  = 0;
  ///! The centre of gravity of robot
  virtual void cog(Eigen::Vector3d&)         = 0;

public:
  ///! The translation of LegType shoulder(leg_base frame) of robot against the world frame.
  virtual void leg_base(LegType, Eigen::Vector3d&) = 0;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_BODY_MATH_BODY_H_ */
