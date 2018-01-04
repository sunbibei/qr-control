/*
 * walk.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: bibei
 */

#include "gait/walk/walk.h"
#include "robot/leg/qr_leg.h"
#include "robot/body/qr_body.h"

#include <toolbox/time_control.h>
#include <foundation/cfg_reader.h>

#ifdef DIS_JNT_LIMIT
#include <repository/resource/joint_manager.h>
#endif

namespace qr_control {

#define FOR_EACH_LEG(l) for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR})
#define FOR_EACH_JNT(j) for (const auto& j : {JntType::YAW, JntType::HIP, JntType::KNEE})

#define PRINT_PARAM_CURRENT_POS \
    leg_ifaces_[LegType::FL]->joint_position_const_ref(), \
    leg_ifaces_[LegType::FR]->joint_position_const_ref(), \
    leg_ifaces_[LegType::HL]->joint_position_const_ref(), \
    leg_ifaces_[LegType::HR]->joint_position_const_ref()

#define PRINT_PARAM_TARGET_POS \
    leg_cmds_[LegType::FL]->target, \
    leg_cmds_[LegType::FR]->target, \
    leg_cmds_[LegType::HL]->target, \
    leg_cmds_[LegType::HR]->target

#define PRINT_PARAM_CURRENT_EEF \
  leg_ifaces_[LegType::FL]->eef(), \
  leg_ifaces_[LegType::FR]->eef(), \
  leg_ifaces_[LegType::HL]->eef(), \
  leg_ifaces_[LegType::HR]->eef()

#define PRINT_CURRENT_POSS   __print_positions(PRINT_PARAM_CURRENT_POS);
#define PRINT_POSS_VS_TARGET __print_positions(PRINT_PARAM_CURRENT_POS, PRINT_PARAM_TARGET_POS);
#define PRINT_CURRENT_EEF    __print_positions(PRINT_PARAM_CURRENT_EEF);
#define PRINT_COMMAND        __print_command(leg_cmds_);

#define PRESS_THEN_GO        {LOG_WARNING << "Press any key to continue."; getchar();}

///! These are the inline functions forward declare.
///! print joint position of the single leg
void __print_positions(const Eigen::VectorXd&);
///! print the v.s. result and different between joint position of the single leg
void __print_positions(const Eigen::VectorXd& _jnt, const Eigen::VectorXd& _tjnt);
///! print FPT(foot point) or COG(centre of gravity) coordinate.
void __print_positions(const Eigen::Vector3d& _xyz);
///! print the v.s. result and different between FPT or COG
void __print_positions(const Eigen::Vector3d& _xyz, const Eigen::Vector3d& _txyz);
///! print the joint position of the all of leg.
void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr);
///! print the PFT of the all of leg.
void __print_positions(const Eigen::Vector3d& fl, const Eigen::Vector3d& fr,
    const Eigen::Vector3d& hl, const Eigen::Vector3d& hr);
///! print the v.s. result and different between FPT or COG
void __print_positions(const Eigen::Vector3d& fl, const Eigen::Vector3d& fr,
    const Eigen::Vector3d& hl, const Eigen::Vector3d& hr,
    const Eigen::Vector3d& tfl, const Eigen::Vector3d& tfr,
        const Eigen::Vector3d& thl, const Eigen::Vector3d& thr);
///! print the v.s. result and different between joint position of the all of leg.
void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr,
    const Eigen::VectorXd& tfl, const Eigen::VectorXd& tfr,
    const Eigen::VectorXd& thl, const Eigen::VectorXd& thr);
///! print the joint commands.
void __print_command(LegTarget**);
///! print the joint commands. (Deprecated)
void __print_command(const Eigen::VectorXd*);

Eigen::Vector2d __incenter(const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);

void __cycloid_position(
      const Eigen::Vector3d&, const Eigen::Vector3d&, int, int, int, Eigen::Vector3d&);

Eigen::Vector2d __cross_point(
    const Eigen::Vector2d& p0_0, const Eigen::Vector2d& p0_1,
    const Eigen::Vector2d& p1_0, const Eigen::Vector2d& p1_1);

double __inscribed_circle_radius(
    const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);

Eigen::Vector2d __inner_heart(
    const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);

Eigen::Vector2d __line_section(
    const Eigen::Vector2d& A, const Eigen::Vector2d& B, double ratio);

// LegType __same_side(LegType _c);
//LegType __ipsilateral(LegType _c);

struct WalkParam {
  ///! The threshold for cog
  double THRES_COG;
  ///! The foot step
  double FOOT_STEP;
  ///! The height value when the robot stay stance.
  double STANCE_HEIGHT;
  ///! The height value when swing leg.
  double SWING_HEIGHT;
  ///! The orientation of forward walk.
  double FORWARD_ALPHA;

  ///! The time for moving COG(in ms)
  int64_t   COG_TIME;
  ///! The time for swing leg (in ms)
  int64_t   SWING_TIME;

  WalkParam(const MiiString& _tag)
    : THRES_COG(6.5),    FOOT_STEP(10),
      STANCE_HEIGHT(46), SWING_HEIGHT(5),
      FORWARD_ALPHA(0),
      COG_TIME(2000),    SWING_TIME(2000) {
    auto cfg = MiiCfgReader::instance();
    cfg->get_value(_tag, "cog_threshold", THRES_COG);
    cfg->get_value(_tag, "step",         FOOT_STEP);
    cfg->get_value(_tag, "stance_height",STANCE_HEIGHT);
    cfg->get_value(_tag, "swing_height", SWING_HEIGHT);
    cfg->get_value(_tag, "forward_orientation", FORWARD_ALPHA);

    cfg->get_value(_tag, "cog_time",     COG_TIME);
    cfg->get_value(_tag, "swing_time",   SWING_TIME);
  }
};

Walk::Walk()
  : current_state_(WalkState::UNKNOWN_WK_STATE),
    body_iface_(nullptr),
    params_(nullptr), timer_(nullptr), eef_traj_(nullptr),
    swing_leg_(LegType::UNKNOWN_LEG), post_tick_interval_(50)
{
  for (auto& iface : leg_ifaces_)
    iface = nullptr;

  for (auto& c : leg_cmds_)
    c = nullptr;

  for (auto& t : cog2eef_traj_)
    t = nullptr;
//  Loop_Count = 0;

#ifdef PUB_ROS_TOPIC
  nh_.reset(new ros::NodeHandle("~"));
#endif
}

Walk::~Walk() {
#ifdef PUB_ROS_TOPIC
  nh_.reset();
  cmd_pub_.reset();
#endif

  delete timer_;
  timer_ = nullptr;

  for (auto& iface : leg_ifaces_)
    iface = nullptr;

  for (auto& c : leg_cmds_) {
    delete c;
    c = nullptr;
  }
}

bool Walk::init() {
  if (!GaitBase::init()) return false;

  auto cfg = MiiCfgReader::instance();
  // cfg->get_value(getLabel(), "hang",     is_hang_walk_);
  cfg->get_value(getLabel(), "interval", post_tick_interval_);

  MiiString label;
  int count      = 0;
  LegType leg    = LegType::UNKNOWN_LEG;
  MiiString _tag = Label::make_label(getLabel(), "leg_iface_" + std::to_string(count));
  while (cfg->get_value(_tag, "leg", leg)) {
    cfg->get_value_fatal(_tag, "label", label);
    auto iface = Label::getHardwareByName<QrLeg>(label);
    if (nullptr == iface) {
      LOG_FATAL << "No such named '" << label << "' interface of leg.";
    } else {
      leg_ifaces_[leg] = iface;
      leg_cmds_[leg] = new LegTarget;
      leg_cmds_[leg]->cmd_type = JntCmdType::CMD_POS;
      leg_cmds_[leg]->target.resize(3);
      leg_cmds_[leg]->target.fill(0.0);
    }

    _tag = Label::make_label(getLabel(), "leg_iface_" + std::to_string(++count));
  }

  _tag = Label::make_label(getLabel(), "body_iface");
  cfg->get_value_fatal(_tag, "label", label);
  auto iface = Label::getHardwareByName<QrBody>(label);
  if (nullptr == iface) {
    LOG_FATAL << "No such named '" << label << "' interface of body.";
  } else {
    body_iface_ = iface;
  }

  if (false) {
    LOG_INFO << "get interface(LegType::FL): " << leg_ifaces_[LegType::FL];
    LOG_INFO << "get interface(LegType::FR): " << leg_ifaces_[LegType::FR];
    LOG_INFO << "get interface(LegType::HL): " << leg_ifaces_[LegType::HL];
    LOG_INFO << "get interface(LegType::HR): " << leg_ifaces_[LegType::HR];
    LOG_INFO << "get interface(Body):        " << body_iface_;
  }

  _tag = Label::make_label(getLabel(), "coefficient");
  params_ = new WalkParam(_tag);

  return true;
}

bool Walk::starting() {
  state_machine_ = new StateMachine<WalkState>(current_state_);
  auto _sm       = (StateMachine<WalkState>*)state_machine_;
  _sm->registerStateCallback(WalkState::WK_INIT_POSE, &Walk::pose_init, this);
  _sm->registerStateCallback(WalkState::WK_MOVE_COG,  &Walk::move_cog,  this);
  _sm->registerStateCallback(WalkState::WK_SWING,     &Walk::swing_leg, this);
  _sm->registerStateCallback(WalkState::WK_SWING_1,   &Walk::close_to_floor, this);
  _sm->registerStateCallback(WalkState::WK_STOP,      &Walk::stance,    this);
  _sm->registerStateCallback(WalkState::WK_SPIRALLING,&Walk::spiralling,this);
  _sm->registerStateCallback(WalkState::WK_HANG,      &Walk::hang_walk, this);

  current_state_ = WalkState::WK_INIT_POSE;
#ifdef PUB_ROS_TOPIC
  cmd_pub_.reset(new realtime_tools::RealtimePublisher<
      std_msgs::Float64MultiArray>(*nh_, "/dragon/joint_commands", 10));
#endif

  timer_    = new TimeControl;
  eef_traj_ = new Trajectory3d;
  for (auto& t : cog2eef_traj_)
    t = new Trajectory3d;

  LOG_INFO << "The walk gait has STARTED!";
  return true;
}

void Walk::stopping() {
  for (auto& t : cog2eef_traj_) {
    delete t;
    t = nullptr;
  }

  delete eef_traj_;
  eef_traj_ = nullptr;

  delete timer_;
  timer_ = nullptr;

  delete state_machine_;
  state_machine_ = nullptr;

  LOG_INFO << "The walk gait has STOPPED!";
}

void Walk::checkState() {
  ///! the local static variable for every state time span.
  static int64_t _s_tmp_span = 0;

  switch (current_state_) {
  case WalkState::WK_INIT_POSE:
  {
    ///! the begin of WK_INIT_POS
    if (!timer_->running()) {
      Eigen::Vector3d _tmp(0, 0, -params_->STANCE_HEIGHT);
      FOR_EACH_LEG(l) {
        leg_ifaces_[l]->inverseKinematics(_tmp, leg_cmds_[l]->target);
      }

      timer_->start();
    }

    if (!end_pose_init()) return;

    ///! the end of WK_INIT_POS
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----INIT POSE OK!("
        << _s_tmp_span << "ms)----*******";
    PRINT_POSS_VS_TARGET

    PRESS_THEN_GO
    ///! updating the following swing leg
    swing_leg_     = next_leg(swing_leg_);
    current_state_ = WalkState::WK_MOVE_COG;
    break;
  }
  case WalkState::WK_MOVE_COG:
  {
    ///! the begin of WK_MOVE_COG
    if (!timer_->running()) {
      prog_cog_traj();

      // PRESS_THEN_GO
      timer_->start();
    }

    if (!end_move_cog()) return;

    ///! the end of WK_MOVE_COG
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----END MVOE  COG("
        << _s_tmp_span << "ms)----*******";

//    Eigen::Vector3d _tfpt = cog2eef_traj_[0]->sample(1);
//    Eigen::Vector3d _fpt(0.0, 0.0, 0.0);
    __print_positions(PRINT_PARAM_CURRENT_EEF,
        cog2eef_traj_[LegType::FL]->sample(cog2eef_traj_[LegType::FL]->ceiling()),
        cog2eef_traj_[LegType::FR]->sample(cog2eef_traj_[LegType::FR]->ceiling()),
        cog2eef_traj_[LegType::HL]->sample(cog2eef_traj_[LegType::HL]->ceiling()),
        cog2eef_traj_[LegType::HR]->sample(cog2eef_traj_[LegType::HR]->ceiling()));
    // __print_positions(_cog, _tcog);

    PRESS_THEN_GO
    current_state_ = WalkState::WK_SWING;
    break;
  }
  case WalkState::WK_SWING:
  case WalkState::WK_SWING_1:
  {
    ///! the begin of WK_SWING
    if (!timer_->running()) {
      // prog_next_fpt();
      prog_eef_traj();
      // sum_interval_ = 0;
      timer_->start();
    }

    auto diff = (eef_traj_->sample(eef_traj_->ceiling()) - leg_ifaces_[swing_leg_]->eef()).norm();
    if (diff < 0.1) {
      ctf_eef_     = eef_traj_->sample(eef_traj_->ceiling());
      ctf_eef_.z() = leg_ifaces_[LEGTYPE_SL(swing_leg_)]->eef().z();
      current_state_ = WalkState::WK_SWING_1;
    }

    if (!end_swing_leg()) return;

    ///! the end of WK_SWING
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----END SWING LEG("
        << _s_tmp_span << "ms)----*******";

    __print_positions(leg_ifaces_[swing_leg_]->eef(), eef_traj_->sample(1));
    PRESS_THEN_GO

    Eigen::Vector3d _fpt = eef_traj_->sample(1);
    _fpt.z() -= 0.1*params_->SWING_HEIGHT;
    leg_ifaces_[swing_leg_]->inverseKinematics(
        _fpt, leg_cmds_[swing_leg_]->target);

    ///! Every twice swing leg then adjusting COG.
    if ((LegType::FL == swing_leg_) || (LegType::FR == swing_leg_)) {
      current_state_ = WalkState::WK_MOVE_COG;
    } else
      current_state_ = WalkState::WK_SWING;

    ///! program the next swing leg
    swing_leg_ = next_leg(swing_leg_);
    break;
  }
  case WalkState::WK_STOP:
  {
    ///! the begin of WK_SWING
    if (!timer_->running()) {
      timer_->start();
    }

    ///! the end of WK_SWING
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----END STOP("
        << _s_tmp_span << "ms)----*******";

    PRINT_CURRENT_EEF
    PRESS_THEN_GO
    break;
  }
  case WalkState::WK_SPIRALLING:
  case WalkState::WK_HANG:
  default:
    LOG_ERROR << "What fucking walk state!";
    break;
  // Nothing to do here.
  }
}

//void Walk::prev_tick() {
//  // gesture->updateImuData(imu_quat_[0], imu_quat_[1], imu_quat_[2]);
//}

void Walk::post_tick() {
  ///! The command frequency control
  static TimeControl _s_post_tick(true);
  static int64_t     _s_sum_interval = 0;
  _s_sum_interval += _s_post_tick.dt();
  if (_s_sum_interval < post_tick_interval_) return;
  _s_sum_interval = 0;

  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[leg]->legTarget(*leg_cmds_[leg]);
    leg_ifaces_[leg]->move();
  }


//  if (current_state_ == WalkState::WK_SWING) {
//    __print_positions(leg_ifaces_[swing_leg_]->eef(), eef_traj_->sample(1));
//  } else
    PRINT_POSS_VS_TARGET// PRINT_COMMAND

#ifdef PUB_ROS_TOPIC
  if(cmd_pub_->trylock()) {
    cmd_pub_->msg_.data.clear();
    for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
      const auto& cmds = jnts_pos_cmd_[l];
      for (const auto& j : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
        cmd_pub_->msg_.data.push_back(cmds(j));
      }
    }
    cmd_pub_->unlockAndPublish();
  }
#endif
}

void Walk::pose_init() {
  // Nothing to do here.
  PRINT_POSS_VS_TARGET
}

bool Walk::end_pose_init() {

  FOR_EACH_LEG(l) {
    auto diff = (leg_ifaces_[l]->joint_position_const_ref()
        - leg_cmds_[l]->target).norm();
    ///! The different of any leg is bigger than 0.1 is considered as
    ///! that the robot has not reach the initialization position.
    if (diff > 0.1) return false;
  }
  return true;
}

///! The flow of swing leg
///! This method only be called after the moving COG
LegType Walk::next_leg(const LegType curr) {
  switch (curr) {
  case LegType::FL: return LegType::HR;
  case LegType::FR: return LegType::HL;
  case LegType::HL: return LegType::FL;
  case LegType::HR: return LegType::FR;
  default: return LegType::HL; ///! The first swing HL leg.
  }
}

void Walk::hang_walk() {
  LOG_ERROR << "NO IMPLEMENT!";
  PRESS_THEN_GO
}

void Walk::move_cog() {
  if (!timer_->running())
    return;

  FOR_EACH_LEG(l) {
    leg_ifaces_[l]->inverseKinematics(
        cog2eef_traj_[l]->sample((double)timer_->span()/params_->COG_TIME),
        leg_cmds_[l]->target);
  }
}

bool Walk::end_move_cog() {
  return (timer_->span() > params_->COG_TIME);
}

void Walk::swing_leg() {
  if (!timer_->running())
    return;

//  auto diff = (eef_traj_->sample(eef_traj_->ceiling()) - leg_ifaces_[swing_leg_]->eef()).norm();
//  if (diff > 0.1) {
    leg_ifaces_[swing_leg_]->inverseKinematics(
        eef_traj_->sample((double)timer_->span()/params_->SWING_TIME),
        leg_cmds_[swing_leg_]->target);
//  } else {
//    close_to_floor();
//  }
}

void Walk::close_to_floor() {
  if (std::abs(leg_ifaces_[swing_leg_]->eef().z() - ctf_eef_.z()) < 0.01) {
    ctf_eef_.z() = ctf_eef_.z() + 0.01;
  }
  // LOG_WARNING << "close to floor";
  ///! setting command.
  leg_ifaces_[swing_leg_]->inverseKinematics(ctf_eef_, leg_cmds_[swing_leg_]->target);
}

bool Walk::end_swing_leg() {
  auto diff = (eef_traj_->sample(eef_traj_->ceiling()) - leg_ifaces_[swing_leg_]->eef()).norm();

  ///! for real robot
  return ((diff < 1) && (LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state()));
  ///! for rviz
  // return ((LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state()) || (timer_->span() > 2*params_->SWING_TIME));

}

// TODO
void Walk::stance() {
  FOR_EACH_LEG(l) {
    ;
  }
}

bool Walk::end_stance() {
  Eigen::Vector3d _tmp(0.0, 0.0, params_->STANCE_HEIGHT);
  // ;
  FOR_EACH_LEG(l) {
    auto diff  = (cog2eef_traj_[l]->sample(1) - leg_ifaces_[l]->eef()).norm();
    if (diff > 0.1) return false;
  }
  return true;
}

// TODO
void Walk::spiralling() {
  ;
}

void Walk::prog_eef_traj() {
  Eigen::Vector3d _last_fpt = leg_ifaces_[swing_leg_]->eef();
  Eigen::Vector3d _next_fpt = prog_next_fpt(swing_leg_);

  Eigen::MatrixXd A;
  A.resize(5, 5);
  ///!  X = a0 + a1 *t + a2 * t^2 + a3 * t^3 + a4 * t^4
  ///!  X'(0) = 0; X'(1) = 0; X(0) = p0; X(1) = p1; X(0.5) = ptop;
  A << 0,   1,    0,     0,      0,
       0,   1,    2,     3,      4,
       1,   0,    0,     0,      0,
       1,   1,    1,     1,      1,
       1, 0.5, 0.25, 0.125, 0.0625;

  Eigen::MatrixXd b;
  b.resize(5, 3);
  b.row(0) << 0, 0, 0;
  b.row(1) << 0, 0, 0;
  b.row(2) = _last_fpt;
  b.row(3) = _next_fpt;
  b.row(4) << (_last_fpt.x()/2 + _next_fpt.x()/2), (_last_fpt.y()/2 + _next_fpt.y()/2), (_last_fpt.z() + params_->SWING_HEIGHT);

  Eigen::MatrixXd c;
  if (0 == A.determinant()) {
    c = A.householderQr().solve(b);
    std::cout << "NO cross point, Using this result: " << c.transpose() << std::endl;
  } else {
    c = A.partialPivLu().solve(b);
  }

  eef_traj_->range(0, 1);
  eef_traj_->reset(c.transpose());
  std::cout << "Trajectory:\n" << *eef_traj_ << std::endl;
}

Eigen::Vector3d Walk::prog_next_fpt(LegType _fsl) {
  Eigen::Vector3d _last_fpt = leg_ifaces_[_fsl]->eef();
  Eigen::Vector3d _next_fpt = _last_fpt;
  Eigen::Vector3d _other_fpt= _last_fpt;

  auto _other_leg = LEGTYPE_SL(_fsl);
  if (LegType::UNKNOWN_LEG != _other_leg)
    _other_fpt = leg_ifaces_[_other_leg]->eef();

  if (std::abs(_other_fpt.x() - _last_fpt.x()) > 0.5*params_->FOOT_STEP) {
    _next_fpt.x() = _other_fpt.x() + params_->FOOT_STEP;
    _next_fpt.y() = _other_fpt.y();
  } else {
    _next_fpt.x() += params_->FOOT_STEP;
    _next_fpt.y()  = _last_fpt.y() + std::abs(_next_fpt.x() - _last_fpt.x()) * tan(params_->FORWARD_ALPHA);
  }
  _next_fpt.z() = _last_fpt.z() + 0.1*params_->SWING_HEIGHT;

  return _next_fpt;
}

void Walk::prog_cog_traj() {
  Eigen::Vector3d _p0(0.0, 0.0, -params_->STANCE_HEIGHT);
  Eigen::Vector3d _p1(0.0, 0.0, -params_->STANCE_HEIGHT);
  Eigen::Vector2d _tmp_next_cog = prog_next_cog(swing_leg_);

  FOR_EACH_LEG(l) {
    _p0 = leg_ifaces_[l]->eef();
    _p1.head(2) = _p0.head(2) - _tmp_next_cog.head(2);

    Eigen::MatrixXd A;
    A.resize(4, 4);
    ///!  X = a0 + a1 *t + a2 * t^2 + a3 * t^3
    ///!  X'(0) = 0; X'(1) = 0; X(0) = p0; X(1) = p1;
    A << 0,   1,    0,     0,
         0,   1,    2,     3,
         1,   0,    0,     0,
         1,   1,    1,     1;

    Eigen::MatrixXd b;
    b.resize(4, 3);
    b.row(0) << 0, 0, 0;
    b.row(1) << 0, 0, 0;
    b.row(2) = _p0;
    b.row(3) = _p1;

    Eigen::MatrixXd c;
    if (0 == A.determinant()) {
      c = A.householderQr().solve(b);
      LOG_ERROR << "NO cross point, Using this result: \n" << c.transpose();
    } else {
      c = A.partialPivLu().solve(b);
    }

    cog2eef_traj_[l]->range(0, 1);
    cog2eef_traj_[l]->reset(c.transpose());
    std::cout << *cog2eef_traj_[l] << std::endl;
  }
}

Eigen::Vector2d Walk::prog_next_cog(LegType _fsl) {
  Eigen::Vector2d _next_cog_proj(0.0, 0.0), _p[3];
  Eigen::Vector2d _last_cog_proj(0.0, 0.0), _lwp[LegType::N_LEGS];
  LegType _next_leg = _fsl;
  _last_cog_proj    = body_iface_->cog().head(2);


  FOR_EACH_LEG(l) {
    if (l != _next_leg)
      _lwp[l] = (leg_ifaces_[l]->eef() + body_iface_->leg_base(l)).head(2);
    else
      _lwp[l] = (prog_next_fpt(_fsl).head(2) + body_iface_->leg_base(l).head(2));
  }
  _p[0] = __cross_point(_lwp[LegType::FL], _lwp[LegType::HR],
      _lwp[LegType::FR], _lwp[LegType::HL]);


  // _p[0] = body_iface_->cog().head(2);
  if ((LegType::FL == _next_leg) || (LegType::HL == _next_leg)) {
    _p[1] = (leg_ifaces_[LegType::FR]->eef() + body_iface_->leg_base(LegType::FR)).head(2);
    _p[2] = (leg_ifaces_[LegType::HR]->eef() + body_iface_->leg_base(LegType::HR)).head(2);
  } else {
    _p[1] = (leg_ifaces_[LegType::FL]->eef() + body_iface_->leg_base(LegType::FL)).head(2);
    _p[2] = (leg_ifaces_[LegType::HL]->eef() + body_iface_->leg_base(LegType::HL)).head(2);
  }
  _next_cog_proj = __incenter(_p[0], _p[1], _p[2]);
  return _next_cog_proj;

  int idx = 0;
  FOR_EACH_LEG(l) {
    if (l == _next_leg) continue;
    _p[idx++] = (leg_ifaces_[l]->eef()
        + body_iface_->leg_base(l)).head(2);
  }

  return __incenter(_p[0], _p[1], _p[2]);
}

// StateMachineBase* Walk::state_machine() { return state_machine_; }

// TODO
void Walk::walk() { ; }

//Eigen::Vector2d Walk::cog_proj1() {
//  if (LegType::UNKNOWN_LEG == swing_leg_)
//    return Eigen::Vector2d(0.0, 0.0);
//
//  double _x = 0;
//  double _y = 0;
//  Eigen::Vector2d _eef_proj(0.0, 0.0);
//  FOR_EACH_LEG(l) {
//    if (swing_leg_ == l) continue;
//    _eef_proj = (leg_ifaces_[l]->eef()
//        + body_iface_->leg_base(l)).head(2);
//    _x += _eef_proj.x();
//    _y += _eef_proj.y();
//  }
//
//  return Eigen::Vector2d(_x, _y);
//
//  Eigen::Vector2d _cs, _p0, _p1, _p2, _p3;
//  _p2 = (leg_ifaces_[LegType::FL]->eef()
//      + body_iface_->leg_base(LegType::FL)).head(2);
//
//  _p3 = (leg_ifaces_[LegType::HR]->eef()
//      + body_iface_->leg_base(LegType::HR)).head(2);
//
//  _p0 = (leg_ifaces_[LegType::FR]->eef()
//      + body_iface_->leg_base(LegType::FR)).head(2);
//
//  _p1 = (leg_ifaces_[LegType::HL]->eef()
//      + body_iface_->leg_base(LegType::HL)).head(2);
//
//  return _cs;
//}

//void Walk::prog_next_fpt() {
//  last_foot_pos_ = leg_ifaces_[swing_leg_]->eef();
//  // last_foot_pos_ = foots_pos_[swing_leg_];
//  // next_foot_pos_ << coeff_->FOOT_STEP, last_foot_pos_.y(), -coeff_->STANCE_HEIGHT;
//  next_foot_pos_ = last_foot_pos_;
//  next_foot_pos_.x() += coeff_->FOOT_STEP;
//}

// Eigen::Vector2d Walk::prog_next_cog(LegType) {
//  if ((LegType::FL == _swing_leg) || (LegType::HL == _swing_leg)) {
//    _p0 = (leg_ifaces_[LegType::FR]->eef()
//        + body_iface_->leg_base(LegType::FR)).head(2);
//    _p1 = (leg_ifaces_[LegType::HR]->eef()
//        + body_iface_->leg_base(LegType::HR)).head(2);
//    _next_cog_proj = __incentre(_last_cog_proj, _p0, _p1);
//  } else if ((LegType::FL == _swing_leg) || (LegType::HL == _swing_leg)) {
//    _p0 = (leg_ifaces_[LegType::FL]->eef()
//        + body_iface_->leg_base(LegType::FL)).head(2);
//    _p1 = (leg_ifaces_[LegType::HL]->eef()
//              + body_iface_->leg_base(LegType::HL)).head(2);
//    _next_cog_proj = inner_triangle(_last_cog_proj, _p0, _p1);
//  }
//  return _next_cog_proj;

//  if ((LegType::FL == swing_leg_) || (LegType::HL == swing_leg_)) {
//    _p0 = (leg_ifaces_[LegType::FR]->eef()
//        + body_iface_->leg_base(LegType::FR)).head(2);
//    _p1 = (leg_ifaces_[LegType::HR]->eef()
//        + body_iface_->leg_base(LegType::HR)).head(2);
//    _next_cog_proj = inner_triangle(_last_cog_proj,_p0, _p1);
//  } else {
//    _p0 = (leg_ifaces_[LegType::FL]->eef()
//        + body_iface_->leg_base(LegType::FL)).head(2);
//    _p1 = (leg_ifaces_[LegType::HL]->eef()
//              + body_iface_->leg_base(LegType::HL)).head(2);
//    _next_cog_proj = inner_triangle(_last_cog_proj, _p0, _p1);
//  }
//  return _next_cog_proj;

//  Eigen::Vector2d _cs, _p0, _p1, _p2, _p3;
//  Eigen::Vector3d _s;
//
//  switch (leg)
//  {
//    case LegType::FL:
//    case LegType::HL:
//      // TODO
//      // body_iface_->leg_base(LegType::FL, _s);
//      _p0 = (leg_ifaces_[LegType::FL]->eef()
//          + body_iface_->leg_base(LegType::FL)).head(2);
//
//      // body_iface_->leg_base(LegType::HR, _s);
//      _p1 = (leg_ifaces_[LegType::HR]->eef()
//          + body_iface_->leg_base(LegType::HR)).head(2);
//
//      // body_iface_->leg_base(LegType::FR, _s);
//      _p2 = (leg_ifaces_[LegType::FR]->eef()
//          + body_iface_->leg_base(LegType::FR)).head(2);
//
//      // body_iface_->leg_base(LegType::HL, _s);
//      _p3 = (leg_ifaces_[LegType::HL]->eef()
//          + body_iface_->leg_base(LegType::HL)).head(2);
//      break;
//    case LegType::FR:
//    case LegType::HR:
//      // body_iface_->leg_base(LegType::FL, _s);
//      _p2 = (leg_ifaces_[LegType::FL]->eef()
//          + body_iface_->leg_base(LegType::FL)).head(2);
//
//      // body_iface_->leg_base(LegType::HR, _s);
//      _p3 = (leg_ifaces_[LegType::HR]->eef()
//          + body_iface_->leg_base(LegType::HR)).head(2);
//
//      // body_iface_->leg_base(LegType::FR, _s);
//      _p0 = (leg_ifaces_[LegType::FR]->eef()
//          + body_iface_->leg_base(LegType::FR)).head(2);
//
//      // body_iface_->leg_base(LegType::HL, _s);
//      _p1 = (leg_ifaces_[LegType::HL]->eef()
//          + body_iface_->leg_base(LegType::HL)).head(2);
//      break;
//    default:
//      LOG_WARNING << "What fucking code with LEG!";
//      return Eigen::Vector2d(0.0, 0.0);
//  }
//
//  _cs = cog_proj();
//  std::cout << "cs1: " << _cs.transpose() << std::endl;
//
//  __cross_point(_p0, _p1, _p2, _p3, _cs);
//  std::cout << "cs1: " << _cs.transpose() << std::endl;
//
//  Eigen::Vector2d it = inner_triangle(_cs, _p2, _p1);
//
//
//
//  std::cout << "p0: " << _p0.transpose() << std::endl;
//  std::cout << "p1: " << _p1.transpose() << std::endl;
//  std::cout << "p2: " << _p2.transpose() << std::endl;
//  std::cout << "p3: " << _p3.transpose() << std::endl;
//  std::cout << "cs: " << _cs.transpose() << std::endl;
//  std::cout << "it: " << it.transpose() << std::endl;
//
//  return it;

  // return inner_triangle(_cs, _p2, _p1);
// }

//Eigen::Vector2d Walk::inner_triangle(
//    const Eigen::Vector2d& _a, const Eigen::Vector2d& _b, const Eigen::Vector2d& _c) {
//  Eigen::Vector2d _init_cog(0.0, 0.0);
//  double ratio = 1;
//  double radius = __inscribed_circle_radius(_a, _b, _c);
//
//  auto iheart = __inner_heart(_a, _b, _c);
//  if ( radius > coeff_->THRES_COG) {
//    ratio = (radius - coeff_->THRES_COG)/radius;
//    auto _ia = __line_section(_a, iheart, ratio);
//    auto _ib = __line_section(_b, iheart, ratio);
//    auto _ic = __line_section(_c, iheart, ratio);
//
//    Eigen::Vector2d _cs1, _cs2;
//    __cross_point(_ia, _ib, _init_cog, iheart, _cs1);
//    __cross_point(_ia, _ic, _init_cog, iheart, _cs2);
//
//    if (_cs1.x() <= std::max(_ia.x(), _ib.x())
//      && _cs1.x() >= std::min(_ia.x(), _ib.x())) {
//        swing_delta_cog_ = (_ib.x() > _ic.x()) ? _ib - _cs1 : _ic - _cs1;
//        swing_delta_cog_ = swing_delta_cog_ / 2.0;
//        return _cs1;
//    }
//    if (_cs2.x() <= std::max(_ia.x(), _ic.x())
//      && _cs2.x() >= std::min(_ia.x(), _ic.x())) {
//      // std::cout<<"second_cross true"<<std::endl;
//      swing_delta_cog_ = (_ib.x() > _ic.x()) ? _ib - _cs2 : _ic - _cs2;
//      swing_delta_cog_ = swing_delta_cog_ / 2.0;
//      // return _cs2;
//    }
//    return _cs2;
//  } else {
//    return iheart;
//  }
//}

//Eigen::Vector2d Walk::stance_velocity(const Eigen::Vector2d& _target_cog, int64_t _span) {
//  double T  = coeff_->COG_TIME;
//  double dt = _span / T;
//
//  // TODO
//  // Eigen::Vector3d samp = eef2cog_traj_->sample(dt);
//  // return (samp.head(2) - _target_cog);
//
//  Eigen::Vector2d stance_vel(0.0, 0.0);
//  stance_vel.x() = (30 * _target_cog.x() * pow(dt, 4)
//      - 60 * _target_cog.x() * pow(dt, 3)
//      + 30 * _target_cog.x() * pow(dt, 2)) / T;
//
//  stance_vel.y() = (30 * _target_cog.y() * pow(dt, 4)
//      - 60 * _target_cog.y() * pow(dt, 3)
//      + 30 * _target_cog.y() * pow(dt, 2)) / T;
//  return stance_vel;
//}
//
//Eigen::Vector2d Walk::stance_velocity(const Eigen::Vector2d& Adj_vec, int Loop) {
//
//  Eigen::Vector2d stance_vel(0.0, 0.0);
//  stance_vel.x() = 30 * Adj_vec.x() * pow(Loop,4) / pow(50, 5)
//      - 60 * Adj_vec.x() * pow(Loop,3) / pow(50, 4)
//      + 30 * Adj_vec.x() * pow(Loop,2) / pow(50, 3);
//
//  stance_vel.y() = 30 * Adj_vec.y() * pow(Loop,4) / pow(50, 5)
//      - 60 * Adj_vec.y() * pow(Loop,3) / pow(50, 4)
//      + 30 * Adj_vec.y() * pow(Loop,2) / pow(50, 3);
//  return stance_vel;
//}

///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of inline methods         //////////////
///////////////////////////////////////////////////////////////////////////////
void __print_positions(const Eigen::VectorXd& _jnt) {
  printf("___________________________________________\n");
  printf("|    -|   YAW  |   HIP  |  KNEE  |\n");
//printf("|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|\n");
  printf("|    -| %+7.04f| %+7.04f| %+7.04f|\n",
      _jnt(JntType::YAW), _jnt(JntType::HIP), _jnt(JntType::KNEE));
  printf("-------------------------------------------\n");
}

void __print_positions(const Eigen::VectorXd& _jnt, const Eigen::VectorXd& _tjnt) {
  auto diff = (_tjnt - _jnt).norm();
  printf("___________________________________________\n");
  printf("|    -|   YAW  |   HIP  |  KNEE  |  ERROR |\n");
//printf("|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|\n");
  printf("|    -| %+7.04f| %+7.04f| %+7.04f|    -   |\n",
      _jnt(JntType::YAW), _jnt(JntType::HIP), _jnt(JntType::KNEE));
  printf("|    =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|\n",
      _tjnt(JntType::YAW), _tjnt(JntType::HIP), _tjnt(JntType::KNEE), diff);
  printf("-------------------------------------------\n");
}

void __print_positions(const Eigen::Vector3d& _xyz) {
  printf("____________________________________________\n");
  printf("| -|    X    |    Y    |    Z    |  ERROR  |\n");
//printf("| -| +00.0000| +00.0000| +00.0000| +00.0000|\n");
  printf("| -| %+8.04f| %+8.04f| %+8.04f|    -    |\n",
      _xyz.x(), _xyz.y(), _xyz.z());
  printf("--------------------------------------------\n");
}

void __print_positions(const Eigen::Vector3d& _xyz, const Eigen::Vector3d& _txyz) {
  auto diff = (_txyz - _xyz).norm();
  printf("____________________________________________\n");
  printf("| -|    X    |    Y    |    Z    |  ERROR  |\n");
//printf("| -| +00.0000| +00.0000| +00.0000| +00.0000|\n");
  printf("| -| %+8.04f| %+8.04f| %+8.04f|    -    |\n",
      _xyz.x(), _xyz.y(), _xyz.z());
  printf("| =| %+8.04f| %+8.04f| %+8.04f| %+8.04f|\n",
      _txyz.x(), _txyz.y(), _txyz.z(), diff);
  printf("--------------------------------------------\n");
}

void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr) {
  printf("___________________________________________________________________\n");
  printf("|LEG -|   YAW  |   HIP  |  KNEE  |LEG -|   YAW  |   HIP  |  KNEE  |\n");
//printf("LEG -| +0.0000| +0.0000| +0.0000|LEG -| +0.0000| +0.0000| +0.0000|\n");
  printf("| FL -| %+7.04f| %+7.04f| %+7.04f|",
      fl(JntType::YAW), fl(JntType::HIP), fl(JntType::KNEE));
  printf(" FR -| %+7.04f| %+7.04f| %+7.04f|\n",
      fr(JntType::YAW), fr(JntType::HIP), fr(JntType::KNEE));

  printf("| HL -| %+7.04f| %+7.04f| %+7.04f|",
      hl(JntType::YAW), hl(JntType::HIP), hl(JntType::KNEE));
  printf(" HR -| %+7.04f| %+7.04f| %+7.04f|\n",
      hr(JntType::YAW), hr(JntType::HIP), hr(JntType::KNEE));
  printf("-------------------------------------------------------------------\n");
}

void __print_positions(const Eigen::Vector3d& fl, const Eigen::Vector3d& fr,
    const Eigen::Vector3d& hl, const Eigen::Vector3d& hr) {
  printf("___________________________________________________________________\n");
  printf("|LEG -|    X   |    Y    |    Z    |\n");
//printf("LEG -| +00.0000| +00.0000| +00.0000|LEG -| +0.0000| +0.0000| +0.0000|\n");
  printf("| FL -| %+8.04f| %+8.04f| %+8.04f|\n",
      fl.x(), fl.y(), fl.z());
  printf("| FR -| %+8.04f| %+8.04f| %+8.04f|\n",
      fr.x(), fr.y(), fr.z());

  printf("| HL -| %+8.04f| %+8.04f| %+8.04f|\n",
      hl.x(), hl.y(), hl.z());
  printf("| HR -| %+8.04f| %+8.04f| %+8.04f|\n",
      hr.x(), hr.y(), hr.z());
  printf("-------------------------------------------------------------------\n");
}

void __print_positions(const Eigen::Vector3d& fl, const Eigen::Vector3d& fr,
    const Eigen::Vector3d& hl, const Eigen::Vector3d& hr,
    const Eigen::Vector3d& tfl, const Eigen::Vector3d& tfr,
        const Eigen::Vector3d& thl, const Eigen::Vector3d& thr) {
  printf("_____________________________________________________________________________________________\n");
  printf("|LEG  |    X    |    Y    |    Z    |  ERROR  |LEG  |    X    |    Y    |    Z    |  ERROR  |\n");
//printf("|LEG  | +00.0000| +00.0000| +00.0000|+00.0000 |LEG  | +00.0000| +00.0000| +00.0000|+00.0000 |\n");
  printf("| FL -| %+8.04f| %+8.04f| %+8.04f|    -    |",
      fl.x(), fl.y(), fl.z());
  printf(" FR -| %+8.04f| %+8.04f| %+8.04f|    -    |\n",
      fr.x(), fr.y(), fr.z());
  printf("| FL =| %+8.04f| %+8.04f| %+8.04f|%+8.04f |",
      tfl.x(), tfl.y(), tfl.z(), (tfl - fl).norm());
  printf(" FR =| %+8.04f| %+8.04f| %+8.04f|%+8.04f |\n",
      tfr.x(), tfr.y(), tfr.z(), (tfr - fr).norm());

  printf("| HL -| %+8.04f| %+8.04f| %+8.04f|    -    |",
      hl.x(), hl.y(), hl.z());
  printf(" HR -| %+8.04f| %+8.04f| %+8.04f|    -    |\n",
      hr.x(), hr.y(), hr.z());
  printf("| FL =| %+8.04f| %+8.04f| %+8.04f|%+8.04f |",
      thl.x(), thl.y(), thl.z(), (thl - hl).norm());
  printf(" FR =| %+8.04f| %+8.04f| %+8.04f|%+8.04f |\n",
      thr.x(), thr.y(), thr.z(), (thr - hr).norm());
  printf("---------------------------------------------------------------------------------------------\n");
}

void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr,
    const Eigen::VectorXd& tfl, const Eigen::VectorXd& tfr,
        const Eigen::VectorXd& thl, const Eigen::VectorXd& thr) {
//  printf("_____________________________________________________________________________________\n");
//  printf("|LEG -|   YAW  |   HIP  |  KNEE  |  ERROR |LEG -|   YAW  |   HIP  |  KNEE  |  ERROR |\n");
//  printf("|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|\n");
  Eigen::VectorXd js[][LegType::N_LEGS] = {
      {fl, fr, hl, hr}, {tfl, tfr, thl, thr}};
  Eigen::VectorXd tjs[] = {tfl, tfr, thl, thr};

  printf("_____________________________________________________________________________________\n");
  printf("| LEG |   YAW  |   HIP  |  KNEE  |  ERROR | LEG |   YAW  |   HIP  |  KNEE  |  ERROR |\n");
  for (const auto& leg : {LegType::FL, LegType::HL}) {
    for (int l = 0; l < 2; ++l) {
      for (int c = 0; c < 2; ++c) {
        printf("%s", ( (const char* []){"| FL ", " FR ", "| HL ", " HR "}[leg + c] ));
        if (0 == l) printf("-");
        else        printf("=");
        FOR_EACH_JNT(j) {
  #ifdef DIS_JNT_LIMIT
          auto _jnts  = middleware::JointManager::instance();
          const auto _jnt   = _jnts->getJointHandle(LegType(leg + c), JntType(j));
          double _min = _jnt->joint_position_min();
          double _max = _jnt->joint_position_max();
  #endif
          if /*(JntType::HIP == j)*/ (js[l][leg + c](j) <= _min)
            printf("| \033[31;1m%+7.04f\033[0m", js[l][leg + c](j));
          else if /*(JntType::KNEE == j)*/ (js[l][leg](j) >= _max)
            printf("| \033[33;1m%+7.04f\033[0m", js[l][leg + c](j));
          else
            printf("| %+7.04f", js[l][leg + c](j));
        }
        auto diff = (js[1][leg + c] - js[0][leg + c]).norm();
        if (0 == l)
          printf("|    -   |");
        else
          printf("| %+7.04f|", diff);
      }
      printf("\n");
    }
//    for (int c = 0; c < 2; ++c) {
//      printf((const char* []){"| FL -", " FR -", "| HL -", " HR -"}[l + c]);
//      FOR_EACH_JNT(j) {
//#ifdef DIS_JNT_LIMIT
//        auto _jnts  = middleware::JointManager::instance();
//        const auto _jnt   = _jnts->getJointHandle(LegType(l + c), JntType(j));
//        double _min = _jnt->joint_position_min();
//        double _max = _jnt->joint_position_max();
//#endif
//        if /*(JntType::HIP == j)*/ (js[l](j) <= _min)
//          printf("| \033[31;1m%+7.04f\033[0m", js[l](j));
//        else if /*(JntType::KNEE == j)*/ (js[l](j) >= _max)
//          printf("| \033[33;1m%+7.04f\033[0m", js[l](j));
//        else
//          printf("| %+7.04f", js[l](j));
//      }
//      printf("|    -   |");
//    }
//    printf("\n");
//
//    for (int c = 0; c < 2; ++c) {
//      auto diff = (tjs[l+c] - js[l+c]).norm();
//      printf((const char* []){"| FL =", " FR =", "| HL =", " HR ="}[l + c]);
//      FOR_EACH_JNT(j) {
//#ifdef DIS_JNT_LIMIT
//        auto _jnts  = middleware::JointManager::instance();
//        const auto _jnt   = _jnts->getJointHandle(LegType(l + c), JntType(j));
//        double _min = _jnt->joint_position_min();
//        double _max = _jnt->joint_position_max();
//#endif
//        if /*(JntType::YAW == j)*/ (tjs[l](j) <= _min)
//          printf("| \033[31;1m%+7.04f\033[0m", tjs[l](j));
//        else if /*(JntType::HIP == j)*/ (tjs[l](j) >= _max)
//          printf("| \033[33;1m%+7.04f\033[0m", tjs[l](j));
//        else
//          printf("| %+7.04f", tjs[l](j));
//      }
//      printf("| %+7.04f|", diff);
//    }
//    printf("\n");
  }
  printf("-------------------------------------------------------------------------------------\n");
//  printf("\n");
//
//  printf("_____________________________________________________________________________________\n");
//  printf("|LEG -|   YAW  |   HIP  |  KNEE  |  ERROR |LEG -|   YAW  |   HIP  |  KNEE  |  ERROR |\n");
//  printf("| FL -| %+7.04f| %+7.04f| %+7.04f|    -   |",
//      fl(JntType::YAW), fl(JntType::HIP), fl(JntType::KNEE));
//  printf(" FR -| %+7.04f| %+7.04f| %+7.04f|    -   |\n",
//      fr(JntType::YAW), fr(JntType::HIP), fr(JntType::KNEE));
//
//  auto diff1 = (tfl - fl).norm();
//  auto diff2 = (tfr - fr).norm();
//  printf("| FL =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|",
//      tfl(JntType::YAW), tfl(JntType::HIP), tfl(JntType::KNEE), diff1);
//  printf(" FR =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|\n",
//      tfr(JntType::YAW), tfr(JntType::HIP), tfr(JntType::KNEE), diff2);
//
//  printf("| HL -| %+7.04f| %+7.04f| %+7.04f|    -   |",
//      hl(JntType::YAW), hl(JntType::HIP), hl(JntType::KNEE));
//  printf(" HR -| %+7.04f| %+7.04f| %+7.04f|    -   |\n",
//      hr(JntType::YAW), hr(JntType::HIP), hr(JntType::KNEE));
//
//  diff1 = (thl - hl).norm();
//  diff2 = (thr - hr).norm();
//  printf("| HL =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|",
//      thl(JntType::YAW), thl(JntType::HIP), thl(JntType::KNEE), diff1);
//  printf(" HR =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|\n",
//      thr(JntType::YAW), thr(JntType::HIP), thr(JntType::KNEE), diff2);
//  printf("-------------------------------------------------------------------------------------\n");
}

void __print_command(LegTarget** leg_cmds_) {
  printf("_________________________________\n");
  printf("LEG -|   YAW  |   HIP  |  KNEE  |\n");
  printf(" FL -| %+7.04f| %+7.04f| %+7.04f|\n",
      leg_cmds_[LegType::FL]->target(JntType::YAW),
      leg_cmds_[LegType::FL]->target(JntType::HIP),
      leg_cmds_[LegType::FL]->target(JntType::KNEE));

  printf(" FR -| %+7.04f| %+7.04f| %+7.04f|\n",
      leg_cmds_[LegType::FR]->target(JntType::YAW),
      leg_cmds_[LegType::FR]->target(JntType::HIP),
      leg_cmds_[LegType::FR]->target(JntType::KNEE));

  printf(" HL -| %+7.04f| %+7.04f| %+7.04f|\n",
      leg_cmds_[LegType::HL]->target(JntType::YAW),
      leg_cmds_[LegType::HL]->target(JntType::HIP),
      leg_cmds_[LegType::HL]->target(JntType::KNEE));

  printf(" HR -| %+7.04f| %+7.04f| %+7.04f|\n",
      leg_cmds_[LegType::HR]->target(JntType::YAW),
      leg_cmds_[LegType::HR]->target(JntType::HIP),
      leg_cmds_[LegType::HR]->target(JntType::KNEE));
  printf("---------------------------------\n");
}

void __print_command(const Eigen::VectorXd* leg_cmds_) {
  printf("_________________________________\n");
  printf("LEG -|   YAW  |   HIP  |  KNEE  |\n");
  printf(" FL -| %+7.04f| %+7.04f| %+7.04f|\n",
      leg_cmds_[LegType::FL](JntType::YAW),
      leg_cmds_[LegType::FL](JntType::HIP),
      leg_cmds_[LegType::FL](JntType::KNEE));

  printf(" FR -| %+7.04f| %+7.04f| %+7.04f|\n",
      leg_cmds_[LegType::FR](JntType::YAW),
      leg_cmds_[LegType::FR](JntType::HIP),
      leg_cmds_[LegType::FR](JntType::KNEE));

  printf(" HL -| %+7.04f| %+7.04f| %+7.04f|\n",
      leg_cmds_[LegType::HL](JntType::YAW),
      leg_cmds_[LegType::HL](JntType::HIP),
      leg_cmds_[LegType::HL](JntType::KNEE));

  printf(" HR -| %+7.04f| %+7.04f| %+7.04f|\n",
      leg_cmds_[LegType::HR](JntType::YAW),
      leg_cmds_[LegType::HR](JntType::HIP),
      leg_cmds_[LegType::HR](JntType::KNEE));
  printf("---------------------------------\n");
}

Eigen::Vector2d __incenter(
    const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) {
  double a = (B - C).norm();
  double b = (A - C).norm();
  double c = (A - B).norm();
  double sum = a + b + c;
  return Eigen::Vector2d(
      (a*A.x() + b*B.x() + c*C.x())/sum,(a*A.y() + b*B.y() + c*C.y())/sum);
}

/*!
 * @brief This methods calculate the next target position through the start
 *        point _p0, end point _p1, the current/total count(t/T), the brachyaxis
 *        of cycloid(b), the result save to _p. The formula as follow:
 *              | x |   | l0 * (dt - 0.50/pi*sin(2*pi*dt)) |
 *              | y | = | l1 * (dt - 0.50/pi*sin(2*pi*dt)) |
 *              | z |   | 2b * (dt - 0.25/pi*sin(4*pi*dt)) |
 *        where L = [l0, l1] = _p1 - _p0, dt = t/T,
 * @param _p0 The start point
 * @param _p1 The end   point
 * @param t   The current count
 * @param T   The total   count
 * @param b   The brachyaxis
 * @param _p  The result point
 */
void __cycloid_position(
      const Eigen::Vector3d& _p0, const Eigen::Vector3d& _p1,
      int t, int T, int b, Eigen::Vector3d& _p) {
  _p(0) = (_p1(0) - _p0(0)) * ((float)t/(float)T - 1.0/2.0/M_PI*sin(2.0*M_PI*t/T));
  _p(1) = (_p1(1) - _p0(1)) * ((float)t/(float)T - 1.0/2.0/M_PI*sin(2.0*M_PI*t/T));
  if(t<=T/2)
    _p(2) = 2.0 * b * ((float)t/(float)T - 1.0/4.0/M_PI*sin(4.0*M_PI*t/T));
  else if(t<=T)
    _p(2) = 2.0 * b * ((float)(T-t)/(float)T - 1.0/4.0/M_PI*sin(4.0*M_PI*(T-t)/T));
}

Eigen::Vector2d __cross_point(
    const Eigen::Vector2d& p0_0, const Eigen::Vector2d& p0_1,
    const Eigen::Vector2d& p1_0, const Eigen::Vector2d& p1_1) {
  Eigen::Vector2d res;
  Eigen::Matrix2d cof_mat;
  Eigen::Vector2d beta;

  if (p0_0.x() != p0_1.x()) {
    cof_mat(0, 0) = -(p0_1.y() - p0_0.y()) / (p0_1.x() - p0_0.x());
    cof_mat(0, 1) = 1;
  } else {
    cof_mat(0, 0) = 1;
    cof_mat(0, 1) = 0;
  }
  beta(0) = cof_mat.row(0) * p0_0;

  if (p1_0.x() != p1_1.x()) {
    cof_mat(1, 0) = -(p1_1.y() - p1_0.y()) / (p1_1.x() - p1_0.x());
    cof_mat(1, 1) = 1;
  } else {
    cof_mat(1, 0) = 1;
    cof_mat(1, 1) = 0;
  }
  beta(1) = cof_mat.row(1) * p1_0;
  Eigen::Vector2d x(0., 0.);
  if (0 == cof_mat.determinant()) {
    res = cof_mat.householderQr().solve(beta);
    std::cout << "NO cross point, Using the result: " << res.transpose() << std::endl;
  } else {
    res = cof_mat.partialPivLu().solve(beta);
  }

  return res;
}

double __inscribed_circle_radius(
    const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) {
  float c = sqrt(pow((A.x()-B.x()),2) + pow((A.y()-B.y()),2));
  float b = sqrt(pow((A.x()-C.x()),2) + pow((A.y()-C.y()),2));
  float a = sqrt(pow((C.x()-B.x()),2) + pow((C.y()-B.y()),2));
  float p = (a+b+c)/2;
  float s = sqrt(p*(p-a)*(p-b)*(p-c));
  return 2*s/(a+b+c);
}

Eigen::Vector2d __inner_heart(
    const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) {
  double a = 0, b = 0, c = 0;
  Eigen::Vector2d heart(0.0, 0.0);

  a = sqrt(pow(B.x() - C.x(), 2) + pow(B.y() - C.y(), 2));
  b = sqrt(pow(A.x() - C.x(), 2) + pow(A.y() - C.y(), 2));
  c = sqrt(pow(A.x() - B.x(), 2) + pow(A.y() - B.y(), 2));

  heart.x() = (a * A.x() + b * B.x() + c * C.x() ) / (a + b + c);
  heart.y() = (a * A.y() + b * B.y() + c * C.y() ) / (a + b + c);
  return heart;
}

Eigen::Vector2d __line_section(
    const Eigen::Vector2d& A, const Eigen::Vector2d& B, double ratio) {
  Eigen::Vector2d result;
  result.x() = B.x() - ratio * (B.x() - A.x());
  result.y() = B.y() - ratio * (B.y() - A.y());
  return result;
}

//LegType __same_side(LegType _c) {
//  switch (_c) {
//  case LegType::FL: return LegType::FR;
//  case LegType::FR: return LegType::FL;
//  case LegType::HL: return LegType::HR;
//  case LegType::HR: return LegType::HL;
//  default: return LegType::UNKNOWN_LEG;
//  }
//}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::Walk, Label)
