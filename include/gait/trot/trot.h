/*
 * trot.h
 *
 *  Created on: Feb 8, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_TROT_TROT_H_
#define INCLUDE_GAIT_TROT_TROT_H_

#include <gait/gait_base.h>

namespace qr_control {

enum TrotState {
  INVALID_TR_STATE = -1,
  N_TR_STATE
};

class Trot: public GaitBase {
public:
  Trot();
  virtual bool auto_init() override;

  virtual ~Trot();

///! These methods are inherited from super class.
public:
  virtual void checkState() override;
  ///! call it after the state callback
  virtual void post_tick() override;

  virtual bool starting()   override;
  virtual void stopping()   override;

protected:
  ///! The state enumeration for creep.
  TrotState                current_state_;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_TROT_TROT_H_ */
