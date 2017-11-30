/*
 * data_leg.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_DATA_LEG_H_
#define INCLUDE_ROBOT_DATA_LEG_H_

#include <foundation/label.h>
#include <Eigen/Dense>

namespace qr_control {

class DataLeg: public Label {
public:
  DataLeg();
  virtual bool init() override;
  virtual ~DataLeg();

public:
  JntType joint_type();

  double         foot_force();
  const double&  foot_force_const_ref();
  const double*  foot_force_const_pointer();

  Eigen::VectorXd        joint_position();
  const Eigen::VectorXd& joint_position_const_ref();
  const Eigen::VectorXd* joint_position_const_pointer();

  Eigen::VectorXd        joint_velocity();
  const Eigen::VectorXd& joint_velocity_const_ref();
  const Eigen::VectorXd* joint_velocity_const_pointer();

  Eigen::VectorXd        joint_torque();
  const Eigen::VectorXd& joint_torque_const_ref();
  const Eigen::VectorXd* joint_torque_const_pointer();

  // Only get the last command.
  Eigen::VectorXd        joint_command();
  Eigen::VectorXd&       joint_command_ref();
  Eigen::VectorXd*       joint_command_pointer();

protected:
  ///! The type of this leg, reference to utf.h
  JntType                jnt_type_;
  ///! The data of the foot's force sensor.
  const double*          foot_force_;
  ///! The vector of joint position which size is 3.
  const Eigen::VectorXd* jnt_pos_[JntDataType::N_JNT_DATA_TYPES];
  ///! The vector of joint command.
  Eigen::VectorXd*       jnt_cmd_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_DATA_LEG_H_ */
