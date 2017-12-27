/*
 * rl_agent.h
 *
 *  Created on: Dec 21, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_AGENT_RL_AGENT_H_
#define INCLUDE_GAIT_AGENT_RL_AGENT_H_

#include <gait/gait_base.h>
#include <robot_plugin2.h>

namespace qr_control {

typedef rl_agent::RL_STATE AGENT_STATE;

class RLAgent: public GaitBase, public rl_agent::RobotPlugin2 {
public:
  RLAgent(const MiiString& _l = "rl-agent");
  virtual bool init() override;

  virtual ~RLAgent();

///! These methods are inherited from super class.
public:
  virtual void              checkState()    override;
  virtual StateMachineBase* state_machine() override;
  AGENT_STATE               currentState()  const;

///! These methods are the callback of AGENT_STATE.
private:
  ///! RL_STATE_RESET callback
  void reset();
  ///! RL_STATE_TRIAL callback
  void trial();
  ///! RL_STATE_NOTHING callback
  void nothing();

///! The topic callback
protected:
  // Position command callback.
  virtual void resetSubCb(const rl_msgs::PositionCommand::ConstPtr&);
  // Trial command callback.
  virtual void trialSubCb(const rl_msgs::TrialCommand::ConstPtr&);

  ///! The initialization position.
  Eigen::VectorXd target_X_pos_;
  ///! The last joint velocities
  Eigen::VectorXd dX_;
  ///! The command gain
  double          gain_;
  ///! Whether the reset command is send
  bool            is_send_; 

protected:
  ///! The state machine corresponds with CreepState for creep
  StateMachine<AGENT_STATE>*   state_machine_;
  ///! The interface for robot leg.
  class QrLeg*                 trial_leg_ifaces_;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_AGENT_RL_AGENT_H_ */
