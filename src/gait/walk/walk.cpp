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
#include <repository/resource/joint_manager.h>

namespace qr_control {

#define PRINT_PARAM_CURRENT_POS \
    leg_ifaces_[LegType::FL]->joint_position_const_ref(), \
    leg_ifaces_[LegType::FR]->joint_position_const_ref(), \
    leg_ifaces_[LegType::HL]->joint_position_const_ref(), \
    leg_ifaces_[LegType::HR]->joint_position_const_ref()

#define PRINT_PARAM_TARGET_POS \
    jnts_pos_cmd_[LegType::FL], \
    jnts_pos_cmd_[LegType::FR], \
    jnts_pos_cmd_[LegType::HL], \
    jnts_pos_cmd_[LegType::HR]

#define PRINT_CURRENT_POSS   __print_positions(PRINT_PARAM_CURRENT_POS);
#define PRINT_POSS_VS_TARGET __print_positions(PRINT_PARAM_CURRENT_POS, PRINT_PARAM_TARGET_POS);
#define PRINT_COMMAND        __print_command(jnts_pos_cmd_);
#define PRESS_THEN_GO        {LOG_WARNING << "Press any key to continue."; getchar();}

///! These are the inline functions forward declare.
void __print_positions(const Eigen::VectorXd&);

void __print_positions(const Eigen::VectorXd& _jnt, const Eigen::VectorXd& _tjnt);

void __print_positions(const Eigen::Vector3d& _xyz, const Eigen::Vector3d& _txyz);

void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr);

void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr,
    const Eigen::VectorXd& tfl, const Eigen::VectorXd& tfr,
    const Eigen::VectorXd& thl, const Eigen::VectorXd& thr);

void __print_command(const Eigen::VectorXd*);
void __cycloid_position(
      const Eigen::Vector3d&, const Eigen::Vector3d&, int, int, int, Eigen::Vector3d&);

void __cross_point(
    const Eigen::Vector2d& p0_0, const Eigen::Vector2d& p0_1,
    const Eigen::Vector2d& p1_0, const Eigen::Vector2d& p1_1,
    Eigen::Vector2d&);

double __inscribed_circle_radius(
    const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);

Eigen::Vector2d __inner_heart(
    const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);

Eigen::Vector2d __line_section(
    const Eigen::Vector2d& A, const Eigen::Vector2d& B, double ratio);

struct WalkCoeff {
  ///! The threshold for cog
  double THRES_COG;
  ///! The foot step
  double FOOT_STEP;
  ///! The height value when the robot stay stance.
  double STANCE_HEIGHT;
  ///! The time for stance
  int    STANCE_TIME;
  ///! The height value when swing leg.
  double SWING_HEIGHT;
  ///! The time for swing leg
  int    SWING_TIME;

  WalkCoeff(const MiiString& _tag)
    : THRES_COG(6.5),    FOOT_STEP(10),
      STANCE_HEIGHT(46), STANCE_TIME(50),
      SWING_HEIGHT(5),   SWING_TIME(30) {
    auto cfg = MiiCfgReader::instance();
    cfg->get_value(_tag, "cog_threshold", THRES_COG);
    cfg->get_value(_tag, "step",     FOOT_STEP);
    cfg->get_value(_tag, "stance_height",STANCE_HEIGHT);
    cfg->get_value(_tag, "stance_time",  STANCE_TIME);
    cfg->get_value(_tag, "swing_height", SWING_HEIGHT);
    cfg->get_value(_tag, "swing_time",   SWING_TIME);
  }
};

Walk::Walk()
  : current_state_(WalkState::UNKNOWN_WK_STATE),
    state_machine_(nullptr), body_iface_(nullptr),
    is_hang_walk_(false), tick_interval_(50),
    sum_interval_(0), coeff_(nullptr),
    timer_(nullptr), swing_leg_(LegType::HL),
    eef_traj_(nullptr), jnt_traj_(nullptr),
    /* These variables are the private. */
    is_send_init_cmds_(false)
{
  for (auto& iface : leg_ifaces_)
    iface = nullptr;

  for (auto& c : leg_cmds_)
    c = nullptr;

  for (auto& j : jnts_pos_cmd_)
    j.resize(JntType::N_JNTS);

  Loop_Count = 0;

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
  cfg->get_value(getLabel(), "hang",     is_hang_walk_);
  cfg->get_value(getLabel(), "interval", tick_interval_);

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
  coeff_ = new WalkCoeff(_tag);

  return true;
}

bool Walk::starting() {
  state_machine_ = new StateMachine<WalkState>(current_state_);
  state_machine_->registerStateCallback(
      WalkState::WK_WAITING,   &Walk::waiting, this);
  state_machine_->registerStateCallback(
      WalkState::WK_INIT_POSE, &Walk::pose_init, this);
  state_machine_->registerStateCallback(
      WalkState::WK_MOVE_COG,  &Walk::move_cog,  this);
  state_machine_->registerStateCallback(
      WalkState::WK_SWING,     &Walk::swing_leg, this);
  state_machine_->registerStateCallback(
      WalkState::WK_HANG,      &Walk::hang_walk, this);

  current_state_ = WalkState::WK_INIT_POSE;
#ifdef PUB_ROS_TOPIC
  cmd_pub_.reset(new realtime_tools::RealtimePublisher<
      std_msgs::Float64MultiArray>(*nh_, "/dragon/joint_commands", 10));
#endif

  timer_ = new TimeControl;
  timer_->start();
  LOG_INFO << "The walk gait has started!";
  return true;
}

void Walk::checkState() {
  // TODO
  switch (current_state_) {
  case WalkState::WK_INIT_POSE:
  {
    if (!is_send_init_cmds_) return;
    for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
      auto diff = (jnts_pos_cmd_[l] - leg_ifaces_[l]->joint_position_const_ref()).norm();
      if (diff > 0.1) return;
    }
    LOG_WARNING << "INIT POSE OK!";
    PRINT_POSS_VS_TARGET
    // It has reach the target positions.
    // TODO
    current_state_ = ((is_hang_walk_) ? WalkState::WK_MOVE_COG : WalkState::WK_WAITING);
    // current_state_ = ((is_hang_walk_) ? WalkState::WK_HANG : WalkState::WK_WAITING);
    is_send_init_cmds_ = false;
    PRESS_THEN_GO

    Loop_Count = 0;
    break;
  }
  case WalkState::WK_MOVE_COG:
    if (Loop_Count >= coeff_->STANCE_TIME) {
      Loop_Count = 0;
      next_foot_pt();
      std::cout << "last_foot_pos: " <<  last_foot_pos_.transpose() << std::endl;
      std::cout << "next_foot_pos: " <<  next_foot_pos_.transpose() << std::endl;
      eef_trajectory();
      LOG_WARNING << "*******----END MVOE  COG----*******";
      timer_->stop();
      sum_interval_  = 0;
      current_state_ = WalkState::WK_SWING;
    }
    break;
  case WalkState::WK_SWING:
  {
    if (!timer_->is_running()) {
      sum_interval_ = 0;
      timer_->start();
      return;
    }

    auto diff = (next_foot_pos_ - leg_ifaces_[swing_leg_]->eef()).norm();
    if ((LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state())
          || (diff < 0.3)) {
      int64_t span = 0;
      timer_->stop(&span);
      delete eef_traj_;
      eef_traj_ = nullptr;

      LOG_WARNING << "*******----END SWING LEG(" << span << "ms)----*******";
      PRESS_THEN_GO

      swing_leg_ = next_leg(swing_leg_);
      ///! Every twice swing leg then adjusting COG.
      if ((LegType::FL != swing_leg_) && (LegType::FR != swing_leg_)) {
        current_state_ = WalkState::WK_MOVE_COG;
        timer_->start();
        Loop_Count = 0;
      } else {
        sum_interval_  = 0;
        next_foot_pt();
        eef_trajectory();
      }
    }

//    if (Loop_Count >= coeff_->SWING_TIME) {
//      Loop_Count = 0;
//
//      swing_leg_ = next_leg(swing_leg_);
//      LOG_WARNING << "*******----END SWING LEG----*******";
//      PRESS_THEN_GO
//      ///! Every twice swing leg then adjusting COG.
//      if ((LegType::FL != swing_leg_) && (LegType::FR != swing_leg_))
//        current_state_ = WalkState::WK_MOVE_COG;
//    }
    break;
  }
  case WalkState::WK_HANG:
  {
    break;
  }
  default:
    LOG_ERROR << "What fucking walk state!";
    break;
  // Nothing to do here.
  }
}

StateMachineBase* Walk::state_machine() { return state_machine_; }

//void Walk::prev_tick() {
//  // gesture->updateImuData(imu_quat_[0], imu_quat_[1], imu_quat_[2]);
//}

void Walk::post_tick() {
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[leg]->legTarget(JntCmdType::CMD_POS, jnts_pos_cmd_[leg]);
    leg_ifaces_[leg]->move();
  }

  if (nullptr != eef_traj_) {
    // PRINT_POSS_VS_TARGET
    //Eigen::VectorXd _c(3);
    //_c.x() = _f.x(); _c.y() = _f.y(); _c.z() = _f.z();
    __print_positions(leg_ifaces_[swing_leg_]->eef(), next_foot_pos_);
  } else
   PRINT_COMMAND

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

void Walk::waiting() {
  PRESS_THEN_GO
}

void Walk::pose_init() {
  // only send the joint command once.
  if (!is_send_init_cmds_) {
    for (auto& f : foots_pos_) {
      f << 0, 0, -coeff_->STANCE_HEIGHT;
    }
    reverse_kinematics();
  }

  is_send_init_cmds_ = true;
  PRINT_POSS_VS_TARGET
}

///! The flow of swing leg
LegType Walk::next_leg(const LegType curr) {
  switch (curr) {
  case LegType::FL: return LegType::HR;
  case LegType::FR: return LegType::HL;
  case LegType::HL: return LegType::FL;
  case LegType::HR: return LegType::FR;
  default: return LegType::UNKNOWN_LEG;
  }
}

void Walk::hang_walk() {
  LOG_ERROR << "NO IMPLEMENT!";
  PRESS_THEN_GO
}

void Walk::next_foot_pt() {
  last_foot_pos_ = leg_ifaces_[swing_leg_]->eef();
  // last_foot_pos_ = foots_pos_[swing_leg_];
  // next_foot_pos_ << coeff_->FOOT_STEP, last_foot_pos_.y(), -coeff_->STANCE_HEIGHT;
  next_foot_pos_ = last_foot_pos_;
  next_foot_pos_.x() += coeff_->FOOT_STEP;
}

void Walk::move_cog() {
  sum_interval_ += timer_->dt();
  if (sum_interval_ < tick_interval_) return;
  sum_interval_ = 0;

  // PRESS_THEN_GO
  // std::cout << Loop_Count << std::endl;
  if (Loop_Count <= 1) {
    delta_cog_ = delta_cog(swing_leg_);
  }

  Eigen::Vector3d _adj3d(0.0, 0.0, 0.0);
  _adj3d.head(2) = stance_velocity(delta_cog_, Loop_Count);
  for (auto& f : foots_pos_) {
    f -= _adj3d;
  }
  // cog_pos_assign(stance_velocity(delta_cog_, Loop_Count));
  reverse_kinematics();
  ++Loop_Count;
}

void Walk::swing_leg() {
  if (nullptr == eef_traj_) {
    LOG_ERROR << "What fucking trajectory.";
    return;
  }

  if (!timer_->is_running() || (sum_interval_ > 2000))
    return;

  sum_interval_ += timer_->dt();
  auto samp = eef_traj_->sample(sum_interval_/2000.0);

  Eigen::Vector3d _xyz(samp.x(), samp.y(), samp.z());
  leg_ifaces_[swing_leg_]->inverseKinematics(_xyz, jnts_pos_cmd_[swing_leg_]);
  return;

  sum_interval_ += timer_->dt();
  if (sum_interval_ < tick_interval_) return;
  sum_interval_ = 0;

  EV3 foot_vel(0,0,0),joint_vel(0,0,0),joint_pos(0,0,0);
  Eigen::Vector3d s;
  Eigen::Vector2d s2d;
  LegState _td = LegState::AIR_STATE;

  if(Loop_Count <= coeff_->SWING_TIME) {
    if(Loop_Count > coeff_->SWING_TIME/3*2) {
      _td = leg_ifaces_[swing_leg_]->leg_state();
    }
    if (LegState::AIR_STATE == _td) {
      __cycloid_position(last_foot_pos_, next_foot_pos_,
          Loop_Count, coeff_->SWING_TIME, coeff_->SWING_HEIGHT, s);
      foots_pos_[swing_leg_] = last_foot_pos_ + s;
    }
  } else {
    _td = leg_ifaces_[swing_leg_]->leg_state();
    if (LegState::AIR_STATE == _td) {

      foots_pos_[swing_leg_].z() = foots_pos_[swing_leg_].z() - 0.1;
      leg_ifaces_[swing_leg_]->inverseKinematics(foots_pos_[swing_leg_], jnts_pos_cmd_[LegType::FL]);
      LOG_INFO << "ON ground_control is working";
      PRESS_THEN_GO
    }
  }
  reverse_kinematics();
  ++Loop_Count;
}

void Walk::eef_trajectory() {
  // TODO
  const auto& _p0 = last_foot_pos_;
  const auto& _p1 = next_foot_pos_;

  Eigen::Matrix3d A;
  A << 1,   0,    0,
       1,   1,    1,
       1, 0.5, 0.25;
  // std::cout << "A:\n" << A << std::endl;
  Eigen::Matrix3d b;
  b.row(0) = _p0;
  b.row(1) = _p1;
  b.row(2) << (_p0.x()/2 + _p1.x()/2), _p0.y(), (_p0.z() + coeff_->SWING_HEIGHT);
  // std::cout << "b:\n" << b << std::endl;
  Eigen::Matrix3d c;
  if (0 == A.determinant()) {
    c = A.householderQr().solve(b);
    std::cout << "NO cross point, Using this result: " << c.transpose() << std::endl;
  } else {
    c = A.partialPivLu().solve(b);
  }

  delete eef_traj_;
  // Trajectory3d::CoeffMat traj_c;
  // traj_c
  eef_traj_ = new Trajectory3d(c.transpose());
  std::cout << "Trajectory:\n" << *eef_traj_ << std::endl;
  PRESS_THEN_GO
}

Eigen::Vector2d Walk::delta_cog(LegType leg) {
  Eigen::Vector2d _cs, _p0, _p1, _p2, _p3;
  Eigen::Vector3d _s;
  for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[l]->eef(foots_pos_[l]);
  }
  switch (leg)
  {
    case LegType::FL:
    case LegType::HL:
      // TODO
      body_iface_->leg_base(LegType::FL, _s);
      _p0 = (foots_pos_[LegType::FL] + _s).head(2);

      body_iface_->leg_base(LegType::HR, _s);
      _p1 = (foots_pos_[LegType::HR] + _s).head(2);

      body_iface_->leg_base(LegType::FR, _s);
      _p2 = (foots_pos_[LegType::FR] + _s).head(2);

      body_iface_->leg_base(LegType::HL, _s);
      _p3 = (foots_pos_[LegType::HL] + _s).head(2);
      break;
    case LegType::FR:
    case LegType::HR:
      body_iface_->leg_base(LegType::FL, _s);
      _p2 = (foots_pos_[LegType::FL] + _s).head(2);

      body_iface_->leg_base(LegType::HR, _s);
      _p3 = (foots_pos_[LegType::HR] + _s).head(2);

      body_iface_->leg_base(LegType::FR, _s);
      _p0 = (foots_pos_[LegType::FR] + _s).head(2);

      body_iface_->leg_base(LegType::HL, _s);
      _p1 = (foots_pos_[LegType::HL] + _s).head(2);
      break;
    default:
      LOG_WARNING << "What fucking code with LEG!";
      return Eigen::Vector2d(0.0, 0.0);
  }

  __cross_point(_p0, _p1, _p2, _p3, _cs);
  return inner_triangle(_cs, _p2, _p1);
}

Eigen::Vector2d Walk::inner_triangle(
    const Eigen::Vector2d& _a, const Eigen::Vector2d& _b, const Eigen::Vector2d& _c) {
  Eigen::Vector2d _init_cog(0.0, 0.0);
  double ratio = 1;
  double radius = __inscribed_circle_radius(_a, _b, _c);

  auto iheart = __inner_heart(_a, _b, _c);
  if ( radius > coeff_->THRES_COG) {
    ratio = (radius - coeff_->THRES_COG)/radius;
    auto _ia = __line_section(_a, iheart, ratio);
    auto _ib = __line_section(_b, iheart, ratio);
    auto _ic = __line_section(_c, iheart, ratio);

    Eigen::Vector2d _cs1, _cs2;
    __cross_point(_ia, _ib, _init_cog, iheart, _cs1);
    __cross_point(_ia, _ic, _init_cog, iheart, _cs2);

    if (_cs1.x() <= std::max(_ia.x(), _ib.x())
      && _cs1.x() >= std::min(_ia.x(), _ib.x())) {
        swing_delta_cog_ = (_ib.x() > _ic.x()) ? _ib - _cs1 : _ic - _cs1;
        swing_delta_cog_ = swing_delta_cog_ / 2.0;
        return _cs1;
    }
    if (_cs2.x() <= std::max(_ia.x(), _ic.x())
      && _cs2.x() >= std::min(_ia.x(), _ic.x())) {
      // std::cout<<"second_cross true"<<std::endl;
      swing_delta_cog_ = (_ib.x() > _ic.x()) ? _ib - _cs2 : _ic - _cs2;
      swing_delta_cog_ = swing_delta_cog_ / 2.0;
      // return _cs2;
    }
    return _cs2;
  } else {
    return iheart;
  }
}

Eigen::Vector2d Walk::stance_velocity(const Eigen::Vector2d& Adj_vec, int Loop) {
  if (Loop > coeff_->STANCE_TIME) {
    std::cout << "Loop:" << Loop << " Stance_Num:" << coeff_->STANCE_TIME << std::endl;
    LOG_ERROR << ("Time Order wrong while stance");
  }
  Eigen::Vector2d stance_vel(0.0, 0.0);
  stance_vel.x() = 30 * Adj_vec.x() * pow(Loop,4) / pow(coeff_->STANCE_TIME, 5)
      - 60 * Adj_vec.x() * pow(Loop,3) / pow(coeff_->STANCE_TIME, 4)
      + 30 * Adj_vec.x() * pow(Loop,2) / pow(coeff_->STANCE_TIME, 3);
  stance_vel.y() = 30 * Adj_vec.y() * pow(Loop,4) / pow(coeff_->STANCE_TIME, 5)
      - 60 * Adj_vec.y() * pow(Loop,3) / pow(coeff_->STANCE_TIME, 4)
      + 30 * Adj_vec.y() * pow(Loop,2) / pow(coeff_->STANCE_TIME, 3);
  return stance_vel;
}


void Walk::walk() {
  // TODO
  ;
}

/*************************************************************************************************************/
/*******************************************  OLD CODE  ******************************************************/
/*************************************************************************************************************/
void Walk::forward_kinematics() {
  for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[l]->forwardKinematics(foots_pos_[l]);
  }
}

void Walk::reverse_kinematics() {
  for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[l]->inverseKinematics(foots_pos_[l], jnts_pos_cmd_[l]);
  }
}



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

void __print_positions(const Eigen::Vector3d& _xyz, const Eigen::Vector3d& _txyz) {
  auto diff = (_txyz - _xyz).norm();
  printf("____________________________________________\n");
  printf("| -|    X    |    Y    |    Z    |  ERROR  |\n");
//printf("| -| +00.0000| +00.0000| +00.0000| +00.0000|\n");
  printf("| -| %+8.04f| %+8.04f| %+8.04f|    -    |\n",
      _xyz(JntType::YAW), _xyz(JntType::HIP), _xyz(JntType::KNEE));
  printf("| =| %+8.04f| %+8.04f| %+8.04f| %+8.04f|\n",
      _txyz(JntType::YAW), _txyz(JntType::HIP), _txyz(JntType::KNEE), diff);
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

void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr,
    const Eigen::VectorXd& tfl, const Eigen::VectorXd& tfr,
        const Eigen::VectorXd& thl, const Eigen::VectorXd& thr) {
  printf("_____________________________________________________________________________________\n");
  printf("|LEG -|   YAW  |   HIP  |  KNEE  |  ERROR |LEG -|   YAW  |   HIP  |  KNEE  |  ERROR |\n");
//printf("|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|\n");
  printf("| FL -| %+7.04f| %+7.04f| %+7.04f|    -   |",
      fl(JntType::YAW), fl(JntType::HIP), fl(JntType::KNEE));
  printf(" FR -| %+7.04f| %+7.04f| %+7.04f|    -   |\n",
      fr(JntType::YAW), fr(JntType::HIP), fr(JntType::KNEE));

  auto diff1 = (tfl - fl).norm();
  auto diff2 = (tfr - fr).norm();
  printf("| FL =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|",
      tfl(JntType::YAW), tfl(JntType::HIP), tfl(JntType::KNEE), diff1);
  printf(" FR =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|\n",
      tfr(JntType::YAW), tfr(JntType::HIP), tfr(JntType::KNEE), diff2);

  printf("| HL -| %+7.04f| %+7.04f| %+7.04f|    -   |",
      hl(JntType::YAW), hl(JntType::HIP), hl(JntType::KNEE));
  printf(" HR -| %+7.04f| %+7.04f| %+7.04f|    -   |\n",
      hr(JntType::YAW), hr(JntType::HIP), hr(JntType::KNEE));

  diff1 = (thl - hl).norm();
  diff2 = (thr - hr).norm();
  printf("| HL =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|",
      thl(JntType::YAW), thl(JntType::HIP), thl(JntType::KNEE), diff1);
  printf(" HR =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|\n",
      thr(JntType::YAW), thr(JntType::HIP), thr(JntType::KNEE), diff2);
  printf("-------------------------------------------------------------------------------------\n");
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

void __cross_point(
    const Eigen::Vector2d& p0_0, const Eigen::Vector2d& p0_1,
    const Eigen::Vector2d& p1_0, const Eigen::Vector2d& p1_1,
    Eigen::Vector2d& res) {
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

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::Walk, Label)
