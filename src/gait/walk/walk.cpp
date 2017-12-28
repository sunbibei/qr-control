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
    jnts_pos_[LegType::FL], \
    jnts_pos_[LegType::FR], \
    jnts_pos_[LegType::HL], \
    jnts_pos_[LegType::HR]

#define PRINT_CURRENT_POS   __print_positions(PRINT_PARAM_CURRENT_POS);
#define PRINT_POS_VS_TARGET __print_positions(PRINT_PARAM_CURRENT_POS, PRINT_PARAM_TARGET_POS);
#define PRINT_COMMAND       __print_command(jnts_pos_);

///! These are the inline functions forward declare.
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
      STANCE_HEIGHT(46), STANCE_TIME(10),
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
    /* These variables are the private. */
    timer_(nullptr), is_send_init_cmds_(false),
    internal_order_(0), swing_leg_(LegType::HL)
{
  for (auto& iface : leg_ifaces_)
    iface = nullptr;

  for (auto& c : leg_cmds_)
    c = nullptr;

  for (auto& j : jnts_pos_)
    j.resize(JntType::N_JNTS);

  Loop_Count = 0;
}

Walk::~Walk() {
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

  state_machine_ = new StateMachine<WalkState>(current_state_);
  state_machine_->registerStateCallback(
      WalkState::WK_WAITING,   &Walk::waiting, this);
  state_machine_->registerStateCallback(
      WalkState::WK_INIT_POSE, &Walk::pose_init, this);
  state_machine_->registerStateCallback(
      WalkState::WK_SWING,     &Walk::walk,    this);
  state_machine_->registerStateCallback(
      WalkState::WK_HANG,      &Walk::hang_walk, this);

  current_state_ = WalkState::WK_INIT_POSE;

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

//  joint_state_publisher_.reset( new realtime_tools::RealtimePublisher<
//    std_msgs::Float64MultiArray>(n, "/dragon/joint_commands", 10));

  timer_ = new TimeControl;
  timer_->start();
  LOG_INFO << "System Init Succeed!";
  return true;
}

void Walk::checkState() {
  // TODO
  switch (current_state_) {
  case WalkState::WK_INIT_POSE:
  {
    if (!is_send_init_cmds_) return;
    for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
      auto diff = (leg_cmds_[l]->target - leg_ifaces_[l]->joint_position_const_ref()).norm();
      if (diff > 0.1) return;
    }
    LOG_WARNING << "INIT POSE OK!";
    PRINT_POS_VS_TARGET
    // It has reach the target positions.
    // TODO
    current_state_ = ((is_hang_walk_) ? WalkState::WK_HANG : WalkState::WK_WAITING);
    is_send_init_cmds_ = false;
    LOG_WARNING << "The robot has reached the initialization pose, input any key to continue";
    getchar();

    internal_order_ = 1;
    break;
  }
  case WalkState::WK_SWING:
  {
    break;
  }
  case WalkState::WK_HANG:
  {
    break;
  }
  default:
    LOG_ERROR << "What fucking code!";
    break;
  // Nothing to do here.
  }
}

StateMachineBase* Walk::state_machine() { return state_machine_; }

void Walk::prev_tick() {
  // gesture->updateImuData(imu_quat_[0], imu_quat_[1], imu_quat_[2]);
}

void Walk::post_tick() {
  // std::cout << "Walk::post_tick()" << std::endl;
  // command_assign();
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[leg]->legTarget(JntCmdType::CMD_POS, jnts_pos_[leg]);
    leg_ifaces_[leg]->move();
  }

  PRINT_COMMAND
}

void Walk::waiting() {
  LOG_WARNING << "Waiting the user operator, press any key to continue.";
  getchar();
}

void Walk::pose_init() {
  if (!is_send_init_cmds_) {
    for (auto& f : foots_pos_) {
      f << 0, 0, -coeff_->STANCE_HEIGHT;
    }
    reverse_kinematics();
  }

  is_send_init_cmds_ = true;
  PRINT_POS_VS_TARGET
}

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
  ///! Time Control
  sum_interval_ += timer_->dt();
  if (sum_interval_ < tick_interval_) return;
  sum_interval_ = 0;

  switch (internal_order_) {
    case 1:
      Loop_Count = 0;
      internal_order_=2;
      break;
    case 2:
      move_cog();
      if(Loop_Count >= coeff_->STANCE_TIME)
      {
        Loop_Count = 0;
        internal_order_=3;
        next_foot_pt();
        LOG_WARNING << "*******----case 3----*******";
      }
      break;
    case 3: //swing leg
      swing_leg(swing_leg_);
      if (Loop_Count >= coeff_->SWING_TIME) {
        // std::cout<<"flow_control: swing done"<<std::endl;
        Loop_Count = 0;

        internal_order_ = 2;

        swing_leg_ = next_leg(swing_leg_);
        // Leg_Order = (Leg_Order>1)?Leg_Order-2:3-Leg_Order;
        LOG_WARNING << "*******----case 4----*******";
      }
      break;
    default:
      LOG_ERROR << "What fucking control!";
      break;
  } // end switch (Time_Order)
  Loop_Count++;
  // LOG_INFO << "Loop time used: " << timer_->dt() << " ms.";
}

void Walk::next_foot_pt() {
  last_foot_pos_ = foots_pos_[swing_leg_];
  next_foot_pos_ << coeff_->FOOT_STEP, last_foot_pos_.y(), -coeff_->STANCE_HEIGHT;
}

void Walk::move_cog() {
  if (Loop_Count <= 1) {
    delta_cog_ = delta_cog(swing_leg_);
    if ((LegType::FL == swing_leg_) || (LegType::FR == swing_leg_))
      coeff_->STANCE_TIME = 1;
    else
      coeff_->STANCE_TIME = 50;
  }

  cog_pos_assign(stance_velocity(delta_cog_, Loop_Count));
  reverse_kinematics();
}

Eigen::Vector2d Walk::delta_cog(LegType leg) {
  Eigen::Vector2d _cs, _p0, _p1, _p2, _p3;
  Eigen::Vector3d _s;
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

void Walk::swing_leg(const LegType& leg) {
  EV3 foot_vel(0,0,0),joint_vel(0,0,0),joint_pos(0,0,0);
  Eigen::Vector3d s;
  Eigen::Vector2d s2d;
  LegState _td = LegState::AIR_STATE;

  if(Loop_Count <= coeff_->SWING_TIME) {
    if(Loop_Count > coeff_->SWING_TIME/3*2) {
      _td = leg_ifaces_[leg]->leg_state();
      // Leg_On_Ground = foot_contact1->singleFootContactStatus(leg);
    }
    if (LegState::AIR_STATE == _td) {
      __cycloid_position(last_foot_pos_, next_foot_pos_, Loop_Count, coeff_->SWING_TIME, coeff_->SWING_HEIGHT, s);
      foots_pos_[leg] = last_foot_pos_ + s;
    }

    // cog moving
//    if(Loop_Count<=Swing_Num/3*2)
//    {
//      Stance_Num = Swing_Num/3*2;
//      s2d = stance_velocity(swing_delta_cog_, Loop_Count);
//      s1.x = s2d.x(); s1.y = s2d.y(); s1.z = 0;
//      // s1 = get_stance_velocity(swing_adj_CoG, Loop_Count);
//      cog_swing(s1, leg);
//    }
  } else {
    _td = leg_ifaces_[leg]->leg_state();
    // Leg_On_Ground = foot_contact->singleFootContactStatus(leg);
    // if(!Leg_On_Ground) {
    if (LegState::AIR_STATE == _td) {
      on_ground(leg);
      std::cout << "ON ground_control is working" << std::endl;
    }
  }
  reverse_kinematics();
}

void Walk::on_ground(const LegType& l) {
  double err = 0.1;
  // Eigen::Vector3d pos;
  Eigen::VectorXd jnt((int)JntType::N_JNTS);
  foots_pos_[l].z() = foots_pos_[l].z() - err;
  // foots_pos_.lf.z = foots_pos_.lf.z - err;
  // pos << foots_pos_.lf.x, foots_pos_.lf.y, foots_pos_.lf.z;
  leg_ifaces_[l]->inverseKinematics(foots_pos_[l], jnts_pos_[LegType::FL]);
  // __kinematics(foots_pos_[l], -1, jnts_pos_[LegType::FL]);
//  jnts_pos_.lf.pitch = jnt(JntType::YAW);
//  jnts_pos_.lf.hip   = jnt(JntType::HIP);
//  jnts_pos_.lf.knee  = jnt(JntType::KNEE);
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

Eigen::Vector2d Walk::stance_velocity(const Eigen::Vector2d& Adj_vec, unsigned int Loop) {
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
void Walk::forward_kinematics()
{
  for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[l]->forwardKinematics(foots_pos_[l]);
  }
}

void Walk::reverse_kinematics() {
  for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[l]->inverseKinematics(foots_pos_[l], jnts_pos_[l]);
  }
}

void Walk::cog_pos_assign(const Eigen::Vector2d& _adj) {
  Eigen::Vector3d _adj3d(_adj.x(), _adj.y(), 0);
  for (auto& f : foots_pos_) {
    f -= _adj3d;
  }
}



///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of inline methods         ////////////
///////////////////////////////////////////////////////////////////////////////
void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr) {
  printf("___________________________________________________________________\n");
  printf("|LEG -|   YAW  |   HIP  |  KNEE  |LEG -|   YAW  |   HIP  |  KNEE  |\n");
//printf("LEG -| +0.0000| +0.0000| +0.0000|LEG -| +0.0000| +0.0000| +0.0000|\n");
  printf("| FL -| %+01.04f| %+01.04f| %+01.04f|",
      fl(JntType::YAW), fl(JntType::HIP), fl(JntType::KNEE));
  printf(" FR -| %+01.04f| %+01.04f| %+01.04f|\n",
      fr(JntType::YAW), fr(JntType::HIP), fr(JntType::KNEE));

  printf("| HL -| %+01.04f| %+01.04f| %+01.04f|",
      hl(JntType::YAW), hl(JntType::HIP), hl(JntType::KNEE));
  printf(" HR -| %+01.04f| %+01.04f| %+01.04f|\n",
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
  printf("| FL -| %+01.04f| %+01.04f| %+01.04f|    -   |",
      fl(JntType::YAW), fl(JntType::HIP), fl(JntType::KNEE));
  printf(" FR -| %+01.04f| %+01.04f| %+01.04f|    -   |\n",
      fr(JntType::YAW), fr(JntType::HIP), fr(JntType::KNEE));

  auto diff1 = (tfl - fl).norm();
  auto diff2 = (tfr - fr).norm();
  printf("| FL =| %+01.04f| %+01.04f| %+01.04f| %+01.04f|",
      tfl(JntType::YAW), tfl(JntType::HIP), tfl(JntType::KNEE), diff1);
  printf(" FR =| %+01.04f| %+01.04f| %+01.04f| %+01.04f|\n",
      tfr(JntType::YAW), tfr(JntType::HIP), tfr(JntType::KNEE), diff2);

  printf("| HL -| %+01.04f| %+01.04f| %+01.04f|    -   |",
      hl(JntType::YAW), hl(JntType::HIP), hl(JntType::KNEE));
  printf(" HR -| %+01.04f| %+01.04f| %+01.04f|    -   |\n",
      hr(JntType::YAW), hr(JntType::HIP), hr(JntType::KNEE));

  diff1 = (thl - hl).norm();
  diff2 = (thr - hr).norm();
  printf("| HL =| %+01.04f| %+01.04f| %+01.04f| %+01.04f|",
      thl(JntType::YAW), thl(JntType::HIP), thl(JntType::KNEE), diff1);
  printf(" HR =| %+01.04f| %+01.04f| %+01.04f| %+01.04f|\n",
      thr(JntType::YAW), thr(JntType::HIP), thr(JntType::KNEE), diff2);
  printf("-------------------------------------------------------------------------------------\n");
}

void __print_command(const Eigen::VectorXd* leg_cmds_) {
  printf("_________________________________\n");
  printf("LEG -|   YAW  |   HIP  |  KNEE  |\n");
  printf("FL  -| %+01.04f| %+01.04f| %+01.04f|\n",
      leg_cmds_[LegType::FL](JntType::YAW),
      leg_cmds_[LegType::FL](JntType::HIP),
      leg_cmds_[LegType::FL](JntType::KNEE));

  printf("FR  -| %+01.04f| %+01.04f| %+01.04f|\n",
      leg_cmds_[LegType::FR](JntType::YAW),
      leg_cmds_[LegType::FR](JntType::HIP),
      leg_cmds_[LegType::FR](JntType::KNEE));

  printf("HL  -| %+01.04f| %+01.04f| %+01.04f|\n",
      leg_cmds_[LegType::HL](JntType::YAW),
      leg_cmds_[LegType::HL](JntType::HIP),
      leg_cmds_[LegType::HL](JntType::KNEE));

  printf("HR  -| %+01.04f| %+01.04f| %+01.04f|\n",
      leg_cmds_[LegType::HR](JntType::YAW),
      leg_cmds_[LegType::HR](JntType::HIP),
      leg_cmds_[LegType::HR](JntType::KNEE));
  printf("---------------------------------\n");
}

void __cycloid_position(
      const Eigen::Vector3d& _p0, const Eigen::Vector3d& _p1,
      int Loop, int T, int H, Eigen::Vector3d& pos) {
  pos(0) = (_p1(0) - _p0(0)) * ((float)Loop/(float)T - 1.0/2.0/M_PI*sin(2.0*M_PI*Loop/T));
  pos(1) = (_p1(1) - _p0(1)) * ((float)Loop/(float)T - 1.0/2.0/M_PI*sin(2.0*M_PI*Loop/T));

  if(Loop<=T/2)
    pos(2) = 2.0 * H * ((float)Loop/(float)T - 1.0/4.0/M_PI*sin(4.0*M_PI*Loop/T));
  else if(Loop<=T)
    pos(2) = 2.0 * H * ((float)(T-Loop)/(float)T - 1.0/4.0/M_PI*sin(4.0*M_PI*(T-Loop)/T));
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
    std::cout << "NO cross point";
  } else {
    res = cof_mat.colPivHouseholderQr().solve(beta);
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
