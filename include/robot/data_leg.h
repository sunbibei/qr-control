/*
 * data_leg.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_DATA_LEG_H_
#define INCLUDE_ROBOT_DATA_LEG_H_

#include <system/foundation/label.h>
#include "origin/origin.h"

namespace qr_control {

class DataLeg: public Label {
public:
  DataLeg();
  virtual bool init() override;
  virtual ~DataLeg();

protected:
  ///! The type of this leg, reference to utf.h
  JntType                jnt_type_;
  ///! The data of the foot's force sensor.
  const short*           foot_force_;
  ///! The vector of joint position which size is 3.
  const Eigen::VectorXd* jnt_pos_;
  ///! The vector of joint command.
  Eigen::VectorXd*       jnt_cmd_[JntCmdType::N_JNT_CMD_TYPES];
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_DATA_LEG_H_ */
