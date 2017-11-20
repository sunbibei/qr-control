/*
 * gait_base.h
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_GAIT_BASE_H_
#define INCLUDE_GAIT_GAIT_BASE_H_

#include <system/foundation/label.h>

namespace qr_control {

class GaitBase: public Label {
public:
  GaitBase(const MiiString& _l = "gait-base");
  virtual bool init() override;

  virtual ~GaitBase();

public:

protected:
  MiiString gait_name_;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_GAIT_BASE_H_ */
