/*
 * creep.cpp
 *
 *  Created on: Nov 21, 2017
 *      Author: bibei
 */

#include "gait/creep/creep.h"
#include "robot/leg/qr_leg.h"

#include <foundation/cfg_reader.h>

namespace qr_control {

struct __PrivateParam {
  size_t count_init_pose;
  size_t count_cali_imu;
  size_t count_stance;
  size_t count_height;
  size_t count_swing;
  size_t count_height2;

  __PrivateParam(const MiiString& _prefix) {
    auto cfg = MiiCfgReader::instance();
    MiiString label = Label::make_label(_prefix, "count");
    cfg->get_value(label, "init_pose", count_init_pose);
    cfg->get_value(label, "cali_imu",  count_cali_imu);
    cfg->get_value(label, "stance",    count_stance);
    cfg->get_value(label, "height",    count_height);
    cfg->get_value(label, "swing",     count_swing);
    cfg->get_value(label, "height",    count_height2);
  }
};

Creep::Creep(const MiiString& _n)
  : GaitBase(_n), current_state_(CreepState::INVALID_CREEP_STATE),
    state_machine_(nullptr), params_(nullptr),
    loop_count_(0), leg_order_(LegType::UNKNOWN_LEG) {
  for (auto& l : leg_ifaces_)
    l = nullptr;
}

bool Creep::init() {
  if (!GaitBase::init()) return false;
  // auto cfg = MiiCfgReader::instance();
  params_        = new __PrivateParam(getLabel());

  state_machine_ = new StateMachine<CreepState>(current_state_);
  state_machine_->registerStateCallback(
      CreepState::STATE_INIT_POS, &Creep::init_pose, this);
  state_machine_->registerStateCallback(
      CreepState::STATE_IMU,      &Creep::cali_imu,  this);
  state_machine_->registerStateCallback(
      CreepState::STATE_STANCE,   &Creep::stance,    this);
  state_machine_->registerStateCallback(
      CreepState::STATE_HEIGHT,   &Creep::height,    this, &current_state_);
  state_machine_->registerStateCallback(
      CreepState::STATE_SWING,    &Creep::swing,     this, &loop_count_);
  state_machine_->registerStateCallback(
      CreepState::STATE_HEIGHT2,  &Creep::height,    this, nullptr);
  current_state_ = CreepState::STATE_INIT_POS;

  int count      = 0;
  LegType leg    = LegType::UNKNOWN_LEG;
  auto cfg       = MiiCfgReader::instance();
  MiiString _tag = Label::make_label(getLabel(), "interface_" + std::to_string(count));
  while (cfg->get_value(_tag, "leg", leg)) {
    MiiString label;
    cfg->get_value_fatal(_tag, "label", label);
    auto iface = Label::getHardwareByName<QrLeg>(label);
    if (nullptr == iface) {
      LOG_FATAL << "No such named '" << label << "' interface of leg.";
    } else {
      leg_ifaces_[leg] = iface;
    }

    _tag = Label::make_label(getLabel(), "interface_" + std::to_string(++count));
  }

  if (false) {
    LOG_INFO << "get interface(LegType::FL): " << leg_ifaces_[LegType::FL];
    LOG_INFO << "get interface(LegType::FR): " << leg_ifaces_[LegType::FR];
    LOG_INFO << "get interface(LegType::HL): " << leg_ifaces_[LegType::HL];
    LOG_INFO << "get interface(LegType::HR): " << leg_ifaces_[LegType::HR];
  }

  return true;
}

Creep::~Creep() {
  if (state_machine_) {
    delete state_machine_;
    state_machine_ = nullptr;
  }
}

void Creep::checkState() {
  switch(current_state_)
  {
    case CreepState::STATE_INIT_POS:
      // for (int )
    if(loop_count_ >= params_->count_init_pose) {
      loop_count_ = 0;
      LOG_INFO << "Robot has been initialized! Please input 1 to continue:";
      // std::cin>>Time_Order;
      current_state_ = CreepState::STATE_IMU;
    }
    break;

    case CreepState::STATE_IMU:
    if(loop_count_ >= params_->count_cali_imu) {
      loop_count_ = 0;
      current_state_ = CreepState::STATE_STANCE;

      LOG_INFO << "IMU data calibration done!";
    }
    break;

    case CreepState::STATE_STANCE:
    if(loop_count_ >= params_->count_stance) {
      loop_count_ = 0;
      current_state_ = CreepState::STATE_HEIGHT;

      LOG_INFO << "Robot shift CoG done! Next swing leg: " << leg_order_;
    }
    break;

    case CreepState::STATE_HEIGHT:
    if(loop_count_ >= params_->count_height) {
      loop_count_      = 0;
      params_->count_height = 0;
      current_state_   = CreepState::STATE_SWING;
      // TODO
      // assign_next_foot();

      LOG_INFO << "Robot height has been recovery!";
    }
    break;

    case CreepState::STATE_SWING:
    if(false /*foot_contact->isLegOnGround()*/)
    if(loop_count_ >= params_->count_swing) {
      loop_count_ = 0;
      // TODO
      // foot_contact->clear();
      current_state_ = (leg_order_ < 2) ?
          CreepState::STATE_HEIGHT2 : CreepState::STATE_STANCE;

      // TODO
      // Leg_Order = (Leg_Order>1)?Leg_Order-2:3-Leg_Order;

      LOG_INFO << "Robot " << leg_order_ << " Leg swing done! Next shift CoG";
    }
    break;

    case CreepState::STATE_HEIGHT2:
    if(loop_count_ >= params_->count_height) {
      loop_count_ = 0;
      LOG_INFO << "Adjust foot position to avoid hanging!";
      current_state_ = CreepState::STATE_STANCE;
    }
    break;

    default:break;
  }

  ++loop_count_;
}

StateMachineBase* Creep::state_machine() {
  return state_machine_;
}

void Creep::init_pose() {
  LOG_EVERY_N(WARNING, 100) << "I'm " << __FILE__ << " " << __LINE__
      << ", STATE: STATE_INIT_POS";
  for (const auto& leg : leg_ifaces_) {
    auto pos = leg->joint_position();
    auto vel = leg->joint_velocity();
    printf("%01d -- %+01.04f %+01.04f %+01.04f,  %+01.04f %+01.04f %+01.04f\n",
      leg->leg_type(), pos(0), pos(1), pos(2), vel(0), vel(1), vel(2));
  }
}

void Creep::cali_imu() {
  LOG_EVERY_N(WARNING, 100) << "I'm " << __FILE__ << " " << __LINE__
      << ", STATE: STATE_IMU";
}

void Creep::stance() {
  LOG_EVERY_N(WARNING, 100) << "I'm " << __FILE__ << " " << __LINE__
      << ", STATE: STATE_STANCE";
  for (const auto& leg : leg_ifaces_) {
    auto pos = leg->joint_position();
    auto vel = leg->joint_velocity();
    printf("%01d -- %+01.04f %+01.04f %+01.04f,  %+01.04f %+01.04f %+01.04f\n",
      leg->leg_type(), pos(0), pos(1), pos(2), vel(0), vel(1), vel(2));
  }
}

void Creep::height(const CreepState* ps) {
  CreepState state = CreepState::STATE_HEIGHT2;
  if (ps) state = *ps;
  LOG_EVERY_N(WARNING, 100) << "I'm " << __FILE__ << " " << __LINE__
      << ", STATE: STATE_HEIGHT"
      << ", and the current state: " << state;
}

void Creep::swing(const size_t* time_count) {
  LOG_EVERY_N(WARNING, 100) << "I'm " << __FILE__ << " " << __LINE__
      << ", STATE: STATE_SWING"
      << ", and the current time count: " << *time_count;
  for (const auto& leg : leg_ifaces_) {
    auto pos = leg->joint_position();
    auto vel = leg->joint_velocity();
    printf("%01d -- %+01.04f %+01.04f %+01.04f,  %+01.04f %+01.04f %+01.04f\n",
      leg->leg_type(), pos(0), pos(1), pos(2), vel(0), vel(1), vel(2));
  }
}


} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::Creep, Label)
