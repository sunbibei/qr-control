/*
 * creep.h
 *
 *  Created on: Nov 21, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_CREEP_CREEP_H_
#define INCLUDE_GAIT_CREEP_CREEP_H_

#include <gait/gait_base.h>

namespace qr_control {

enum CreepState {
  INVALID_CREEP_STATE = -1,
  STATE_INIT_POS,
  STATE_IMU,
  STATE_STANCE,
  STATE_HEIGHT,
  STATE_SWING,
  STATE_HEIGHT2,
  N_CREEP_STATE
};

class Creep: public GaitBase {
public:
  Creep(const MiiString& _n = "creep");
  virtual bool init() override;

  virtual ~Creep();

///! These methods are inherited from super class.
public:
  virtual void              checkState()    override;
  virtual StateMachineBase* state_machine() override;
  CreepState                currentState()  const;

protected:
  ///! The state enumeration for creep.
  CreepState                current_state_;
  ///! The state machine corresponds with CreepState for creep
  StateMachine<CreepState>* state_machine_;

private:
  ///! Example
  void init_pose();
  void cali_imu();
  void stance();
  void height(const CreepState* state);
  void swing(const size_t* time_count);

private:
  class __PrivateParam* params_;
  size_t                loop_count_;
  LegType               leg_order_;

  class QrLeg*          leg_ifaces_[LegType::N_LEGS];

};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_CREEP_CREEP_H_ */
