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
  SL_TRU_STATE,
  SL_FGT_STATE,
  SL_STP_STATE,
  N_TR_STATE
};

class Trot: public GaitBase {
public:
  Trot();
  virtual bool init() override;

  virtual ~Trot();

///! These methods are inherited from super class.
public:
  virtual void checkState() override;
  ///! call it after the state callback
  virtual void post_tick() override;

  virtual bool starting()   override;
  virtual void stopping()   override;

///! These methods are the callback methods for WalkState.
private:
  ///! callback for SL_TRU_STATE
  void sl_hp_thrust();
  ///! callback for SL_FGT_STATE
  void sl_hp_flight();
  ///! callback for SL_STP_STATE
  void sl_hp_stopping();

protected:
  ///! The state enumeration for trot.
  TrotState             current_state_;
  ///! The private parameters for trot.
  class __TTParams*     trot_params_;
  ///! The interface for body
  class RobotBody*      body_iface_;
  ///! The interface for legs
  class RobotLeg*       leg_ifaces_[LegType::N_LEGS];
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_TROT_TROT_H_ */
