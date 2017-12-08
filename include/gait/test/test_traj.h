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

enum TestTrajState {
  INVALID_TEST_STATE = -1,
  STATE_INIT,
  STATE_READ_STATE,
  STATE_COMMAND,
  STATE_READ_CMD,
  STATE_TRAJ_JNT,
  STATE_TRAJ_EEF,
  N_TEST_TRAJ_STATE
};

class TestTraj: public GaitBase {
public:
  TestTraj(const MiiString& _n = "TestTraj");
  virtual bool init() override;

  virtual ~TestTraj();

///! These methods are inherited from super class.
public:
  virtual void              checkState()    override;
  virtual StateMachineBase* state_machine() override;
  TestTrajState             currentState()  const;

protected:
  ///! The state enumeration for creep.
  TestTrajState                current_state_;
  ///! The state machine corresponds with CreepState for creep
  StateMachine<TestTrajState>* state_machine_;

private:
  ///! Example
  void initialize();
  void print();
  void command();
  void read_cmd();
  void traj();

  // specify code
  void spec_code();

private:
  size_t                loop_count_;
  LegType               leg_order_;

  class QrLeg*          leg_ifaces_[LegType::N_LEGS];

};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_CREEP_CREEP_H_ */
