/*
 * data_leg.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_LEG_DATA_LEG_H_
#define INCLUDE_ROBOT_LEG_DATA_LEG_H_

#include <foundation/label.h>
#include <foundation/utf.h>
#include <Eigen/Dense>

namespace qr_control {

class DataLeg: public Label {
public:
  DataLeg();
  virtual bool init() override;
  virtual ~DataLeg();

public:
  LegType leg_type();

  double         foot_force();
  const double&  foot_force_const_ref();
  const double*  foot_force_const_pointer();

  EVX        joint_position();
  const EVX& joint_position_const_ref();
  const EVX* joint_position_const_pointer();

  EVX        joint_velocity();
  const EVX& joint_velocity_const_ref();
  const EVX* joint_velocity_const_pointer();

  EVX        joint_torque();
  const EVX& joint_torque_const_ref();
  const EVX* joint_torque_const_pointer();

  // Only get the last command.
  EVX        joint_command();
  EVX&       joint_command_ref();
  EVX*       joint_command_pointer();

protected:
  ///! The type of this leg, reference to utf.h
  LegType                leg_type_;
  ///! The data of the foot's force sensor.
  const double*          foot_force_;
  ///! The vector of joint position which size is 3.
  const EVX* jnt_pos_[JntDataType::N_JNT_DATA_TYPES];
  ///! The vector of joint command.
  EVX*       jnt_cmd_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_LEG_DATA_LEG_H_ */
