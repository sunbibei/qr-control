/*
 * trot.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: bibei
 */

#include <gait/trot/trot.h>
#include "robot/qr_robot.h"

#include <foundation/cfg_reader.h>

namespace qr_control {

///! The private parameters for trot.
struct __TTParams {
  ///! The stance height, namely the balance position of spring.
  double STC_LEN;

  __TTParams(const MiiString& _tag) {
    auto cfg = MiiCfgReader::instance();
    cfg->get_value(_tag, "stc_len", STC_LEN);
  }
};

Trot::Trot()
  : current_state_(TrotState::INVALID_TR_STATE),
    trot_params_(nullptr), body_iface_(nullptr) {
  // TODO Auto-generated constructor stub

}

bool Trot::init() {
  if (!GaitBase::init()) return false;

  auto ifaces = LegRobot::instance();
  if (!ifaces) {
    LOG_FATAL << "The interface for robot is null!";
    return false;
  }
  body_iface_ = ifaces->robot_body();
  if (!body_iface_)
    LOG_FATAL << "The interface of RobotBody is null!";

  FOR_EACH_LEG(l) {
    leg_ifaces_[l] = ifaces->robot_leg(l);
    if (!leg_ifaces_[l])
      LOG_FATAL << "The interface of RobotLeg" << LEGTYPE_TOSTRING(l) << " is null!";
  }

  ///! TODO
  auto cfg = MiiCfgReader::instance();

  return true;
}

Trot::~Trot() {
  stopping();
}

bool Trot::starting() {
  state_machine_ = new StateMachine<TrotState>(current_state_);
  auto _sm       = (StateMachine<TrotState>*)state_machine_;
  _sm->registerStateCallback(TrotState::SL_TRU_STATE, &Trot::sl_hp_thrust,   this);
  _sm->registerStateCallback(TrotState::SL_FGT_STATE, &Trot::sl_hp_flight,   this);
  _sm->registerStateCallback(TrotState::SL_STP_STATE, &Trot::sl_hp_stopping, this);
  // TODO
  trot_params_     = new __TTParams(Label::make_label(getLabel(), "parameters"));

  return true;
}

void Trot::stopping() {
  current_state_ = TrotState::INVALID_TR_STATE;

  delete state_machine_;
  state_machine_ = nullptr;

  delete trot_params_;
  trot_params_   = nullptr;
}

void Trot::checkState() {
  switch(current_state_) {
  case TrotState::SL_TRU_STATE:
    break;
  case TrotState::SL_FGT_STATE:
    break;
  case TrotState::SL_STP_STATE:
    break;
  default:
    LOG_ERROR << "What fucking State!";
  }
}

///! call it after the state callback
void Trot::post_tick() {
  ;
}

///! callback for SL_TRU_STATE
void Trot::sl_hp_thrust() {
  ;
}

///! callback for SL_FGT_STATE
void Trot::sl_hp_flight() {
  ;
}

///! callback for SL_STP_STATE
void Trot::sl_hp_stopping() {
  ;
}

} /* namespace qr_control */
