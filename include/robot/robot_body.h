/*
 * robot_body.h
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_ROBOT_BODY_H_
#define INCLUDE_ROBOT_ROBOT_BODY_H_

#include <foundation/label.h>
#include <Eigen/Dense>

namespace qr_control {

class RobotBody: public Label {
public:
  RobotBody() : Label("robot-body") { };
  virtual ~RobotBody() { };

public:
  /*!
   * @brief Calculate the current position of the center of the main body.
   * @param translation [out] The translation from the base frame.
   * @param quaternion  [out] The quaternion related to the base frame.
   */
  virtual void cog(Eigen::Vector3d& translation) = 0;
  /*!
   * @brief Calculate the current rotation of the main body.
   * @param quaternion  [out] The quaternion representing the rotation from the base frame.
   */
  virtual void rotation(Eigen::Quaterniond& quaternion) = 0;
  /*!
   * @brief Calculate the current velocity of the center of the main body.
   * @param v [out] The current velocity under the base frame.
   */
  virtual void velocity(Eigen::Vector3d& v) = 0;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_ROBOT_BODY_H_ */
