/*
 * rl_agent.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: bibei
 */

#include "gait/agent/rl_agent.h"
#include "robot/leg/qr_leg.h"

#include <foundation/cfg_reader.h>

namespace qr_control {

RLAgent::RLAgent(const MiiString& _l)
  : GaitBase(_l), rl_agent::RobotPlugin2(_l),
    gain_(1.0),
    /*current_state_(AGENT_STATE::UNKONWN_RL_STATE),*/
    state_machine_(nullptr),
    trial_leg_ifaces_(nullptr) {
  ;
}

bool RLAgent::init() {
  if (!GaitBase::init()) return false;

  tag_ = getLabel();
  rl_agent::RobotPlugin2::initialize();

  state_machine_ = new StateMachine<AGENT_STATE>(current_state_);
  state_machine_->registerStateCallback(
      AGENT_STATE::RL_STATE_RESET,   &RLAgent::reset,   this);
  state_machine_->registerStateCallback(
      AGENT_STATE::RL_STATE_TRIAL,   &RLAgent::trial,   this);
  state_machine_->registerStateCallback(
      AGENT_STATE::RL_STATE_NOTHING, &RLAgent::nothing, this);

  current_state_ = AGENT_STATE::RL_STATE_NOTHING;

  auto cfg       = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "gain", gain_);

  MiiString label;
  MiiString _tag = Label::make_label(getLabel(), "interface");
  cfg->get_value_fatal(_tag, "label", label);
  trial_leg_ifaces_ = Label::getHardwareByName<QrLeg>(label);
  if (nullptr == trial_leg_ifaces_) {
    LOG_FATAL << "No such named '" << label << "' interface of leg.";
  }

  if (_DEBUG_INFO_FLAG) {
    LOG_INFO << "get trial interface: " << trial_leg_ifaces_;
  }

  return true;
}

RLAgent::~RLAgent() {
  ;
}

void RLAgent::checkState() {
  switch (current_state_) {
  case AGENT_STATE::RL_STATE_NOTHING:
    break;
  case AGENT_STATE::RL_STATE_TRIAL: // "Trial phase"
  {
    if (trial_controller_.get() && trial_controller_->isFinished()) {
      LOG_WARNING << "This trial has finished.";

      if (is_report_waiting_) {
        // LOG_WARNING << "Is Send Sample?";
        pubSampleReport(samples_, trial_controller_->getTrialLength());
      }

      is_report_waiting_ = false;
      trial_controller_.reset();
      U_.fill(0.0);
      current_state_ = AGENT_STATE::RL_STATE_NOTHING;
    }
    break;
  }
  case AGENT_STATE::RL_STATE_RESET: // "Reset phase"
  {
    samples_->getData(gps::JOINT_ANGLES,     X_ , 0);
    samples_->getData(gps::JOINT_VELOCITIES, dX_, 0);
    if (target_X_pos_.size() != X_.size()) {
      LOG_FATAL << "What fucking code! The size of target_X_pos is different "
          << "between with the size of X_";
      return;
    }

    // Check whether we are close enough to the current target.
    double error = (X_ - target_X_pos_).norm();
    double vel = dX_.norm();
    if (error < 0.1 && vel < 0.1) {
      LOG_WARNING << "Position target has arrived";

      if (is_report_waiting_) {
        pubSampleReport(samples_, 0);
        is_report_waiting_ = false;
      }

      U_.fill(0.0);
      current_state_ = AGENT_STATE::RL_STATE_NOTHING;
    }

    printf("JOINT TARGET: {YAW: %+01.04f HIP: %+01.04f KNEE: %+01.04f}\n",
        target_X_pos_(JntType::YAW), target_X_pos_(JntType::HIP), target_X_pos_(JntType::KNEE));

    printf("JOINT ANGLES: {YAW: %+01.04f HIP: %+01.04f KNEE: %+01.04f ERROR: %+01.04f VEL: %+01.04f}\n",
        X_(JntType::YAW), X_(JntType::HIP), X_(JntType::KNEE), error, vel);

    /*std::cout << "The current joint velocities: " << std::endl;
    for (int i = 0; i < dX_.size(); ++i)
      printf("%+01.04f ", dX_(i));
    printf("\n");*/

//    std::cout << "The current joint angles error: " << std::endl;
//    auto diff = X_ - target_X_pos_;
//    for (int i = 0; i < diff.size(); ++i)
//      printf("%+01.04f ", diff(i));
//    printf("norm: %+01.04f\n\n", error);
  }
  break;
  default:
    LOG_ERROR << "What fucking code!";
    break;
  // Nothing to do here.
  }

  updateSensors();
}

///! RL_STATE_NOTHING callback
void RLAgent::nothing() {
  ;
}

///! RL_STATE_RESET callback
void RLAgent::reset() {
  if (JntType::N_JNTS != target_X_pos_.size()) {
    LOG_ERROR << "Something is wrong, the size of target joints is not match "
        << "between with leg " << trial_leg_ifaces_->leg_type();
    return;
  }

  trial_leg_ifaces_->legTarget(JntCmdType::CMD_POS, target_X_pos_);

//  for (const auto& t : {JntType::YAW, JntType::HIP, JntType::KNEE})
//    trial_leg_ifaces_->jointTarget(t, JntCmdType::CMD_POS, target_X_pos_(t));

  trial_leg_ifaces_->move();
}

///! RL_STATE_TRIAL callback
void RLAgent::trial() {
  if (!trial_controller_.get()) {
    LOG_ERROR << "The trial controller has not configured!";
    return;
  }

  int trial_step_counter = trial_controller_->getStepCounter();
  // LOG_WARNING << "Call GetData to obtain state";
  samples_->getData(trial_controller_->state_types(), X_,
      trial_step_counter);

  // LOG_WARNING << "The Current Step: " << controller_counter_;
  if (!trial_controller_->control(X_, U_)) return;

  U_ *= gain_;
  // Write the command into robot.
  trial_leg_ifaces_->legTarget(JntCmdType::CMD_MOTOR_VEL, U_);
//  for (const auto& t : {JntType::YAW, JntType::HIP, JntType::KNEE})
//    trial_leg_ifaces_->jointTarget(t, JntCmdType::CMD_MOTOR_VEL, U_(t));
  // LOG_WARNING << "Get Action SUCCESSFUL";
  // Update the sample at each time step.
  samples_->setData(gps::ACTION, U_, U_.size(),
      rl_agent::SampleDataFormat::SampleDataFormatEigenVector,
      trial_step_counter);

  ///! the real execute
  trial_leg_ifaces_->move();
}


// Position command callback.
void RLAgent::resetSubCb(const rl_msgs::PositionCommand::ConstPtr& msg) {
  LOG_INFO << "received position command";

  if (JntType::N_JNTS != msg->data.size())
    LOG_FATAL << "ERROR INIT POSE SIZE!";

  target_X_pos_.resize(JntType::N_JNTS);

  std::cout << "init pose: ";
  for (const auto& t : {JntType::YAW, JntType::HIP, JntType::KNEE}) {
    target_X_pos_(t) = msg->data[t];
    printf("%+01.04f ", target_X_pos_(t));
  }
  std::cout << std::endl;

  is_report_waiting_ = true;
  LOG_INFO << "Turn State to RESET";
  current_state_ = AGENT_STATE::RL_STATE_RESET;
}

// Trial command callback.
void RLAgent::trialSubCb(const rl_msgs::TrialCommand::ConstPtr& msg) {
  LOG_INFO << "received trial command";

  rl_agent::CtrlConfigMap controller_params;
  std::vector<gps::SampleType> state_datatypes;
  state_datatypes.reserve(msg->state_datatypes.size());
  for(const auto& type : msg->state_datatypes) {
    state_datatypes.push_back((gps::SampleType)type);
  }
  controller_params["state_include"] = state_datatypes;

  std::vector<gps::SampleType> obs_datatypes;
  obs_datatypes.reserve(msg->obs_datatypes.size());
  for(const auto& type : obs_datatypes) {
    obs_datatypes.push_back(type);
  }
  controller_params["obs_include"] = obs_datatypes;
  // msg::frequency in Hz
  controller_params["ctrl_frequency"] = (int64_t)(1000.0 / msg->frequency);
  // LOG_ERROR << "MSG->FREQUENCY: " << msg->frequency;
  //controller_params["X_include"] = state_inc;

  if(msg->controller.controller_to_execute == gps::LIN_GAUSS_CONTROLLER) {
    configureLqr(msg, controller_params);
  }
  else {
    LOG_ERROR << "Unknown trial controller arm type and/or USE_CAFFE=0/or USE_TF=0";
  }

  // TODO What is funking code!
  // Configure sensor for trial
  rl_agent::DataHandleMap sensor_params;
  // Feed EE points/sites to sensors
  Eigen::MatrixXd ee_points;
  if(0 != msg->ee_points.size() % 3) {
    LOG_ERROR << "Got " << msg->ee_points.size()
        << " %d ee_points (must be multiple of 3)";
  }
  int n_points = msg->ee_points.size() / 3;
  ee_points.resize(n_points, 3);
  for(int i = 0; i < n_points; ++i){
    for(int j = 0; j < 3; ++j){
      ee_points(i, j) = msg->ee_points[j + 3 * i];
    }
  }
  sensor_params["ee_sites"] = ee_points;

  // update end effector points target
  Eigen::MatrixXd ee_points_tgt;
  if(msg->ee_points_tgt.size() != ee_points.size()) {
    LOG_ERROR << "Got " << msg->ee_points_tgt.size()
        << "ee_points_tgt (must match ee_points size: "
        << msg->ee_points.size() << ")";
  }
  ee_points_tgt.resize(n_points, 3);
  for(int i=0; i<n_points; i++) {
    for(int j=0; j<3; j++) {
      ee_points_tgt(i, j) = msg->ee_points_tgt[j+3*i];
    }
  }
  sensor_params["ee_points_tgt"] = ee_points_tgt;
  configureSensors(sensor_params);

  configureSamples(msg->T);
  // Update sensor frequency
  // sensors_->setUpdateFrequency(msg->frequency);

  is_report_waiting_ = true;
  current_state_ = AGENT_STATE::RL_STATE_TRIAL;
}

StateMachineBase* RLAgent::state_machine() {
  return state_machine_;
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::RLAgent, Label)
