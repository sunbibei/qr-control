/*
 * walk.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: bibei
 */

#include "gait/walk/walk.h"

#include <foundation/cfg_reader.h>
#include <repository/resource/joint_manager.h>

namespace qr_control {

#define PRINT_PARAM_CURRENT_POS \
    leg_ifaces_[LegType::FL]->joint_position_const_ref(), \
    leg_ifaces_[LegType::FR]->joint_position_const_ref(), \
    leg_ifaces_[LegType::HL]->joint_position_const_ref(), \
    leg_ifaces_[LegType::HR]->joint_position_const_ref()

#define PRINT_PARAM_TARGET_POS \
    leg_cmds_[LegType::FL].target, \
    leg_cmds_[LegType::FR].target, \
    leg_cmds_[LegType::HL].target, \
    leg_cmds_[LegType::HR].target

#define PRINT_CURRENT_POS   __print_positions(PRINT_PARAM_CURRENT_POS);
#define PRINT_POS_VS_TARGET __print_positions(PRINT_PARAM_CURRENT_POS, PRINT_PARAM_TARGET_POS);

///! These are the inline functions forward declare.
void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr);

void __print_positions(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr,
    const Eigen::VectorXd& tfl, const Eigen::VectorXd& tfr,
    const Eigen::VectorXd& thl, const Eigen::VectorXd& thr);

void __print_command(const LegTarget*);

Walk::Walk()
  : current_state_(WalkState::UNKNOWN_WK_STATE),
    state_machine_(nullptr), is_hang_walk_(false),
    /* These variables are the private. */
    timer_(nullptr), is_send_init_cmds_(false),
    internal_order_(0), swing_leg_(LegType::HL)
{
  for (auto& iface : leg_ifaces_)
    iface = nullptr;

  Loop_Count = 0;
}

Walk::~Walk() {
  delete timer_;
  timer_ = nullptr;

  for (auto& iface : leg_ifaces_)
    iface = nullptr;

  delete foot_contact;
  delete swing;
  delete math;
  delete gesture;
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

  // TODO
  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "hang", is_hang_walk_);

  int count      = 0;
  LegType leg    = LegType::UNKNOWN_LEG;
  MiiString _tag = Label::make_label(getLabel(), "interface_" + std::to_string(count));
  while (cfg->get_value(_tag, "leg", leg)) {
    MiiString label;
    cfg->get_value_fatal(_tag, "label", label);
    auto iface = Label::getHardwareByName<QrLeg>(label);
    if (nullptr == iface) {
      LOG_FATAL << "No such named '" << label << "' interface of leg.";
    } else {
      leg_ifaces_[leg] = iface;
      leg_cmds_[leg].cmd_type = JntCmdType::CMD_POS;
      leg_cmds_[leg].target.resize(3);
      leg_cmds_[leg].target.fill(0.0);
    }

    _tag = Label::make_label(getLabel(), "interface_" + std::to_string(++count));
  }

  if (false) {
    LOG_INFO << "get interface(LegType::FL): " << leg_ifaces_[LegType::FL];
    LOG_INFO << "get interface(LegType::FR): " << leg_ifaces_[LegType::FR];
    LOG_INFO << "get interface(LegType::HL): " << leg_ifaces_[LegType::HL];
    LOG_INFO << "get interface(LegType::HR): " << leg_ifaces_[LegType::HR];
  }

  MiiVector<MiiString> _tds;
  cfg->get_value_fatal(getLabel(), "tds", _tds);
  for (const auto& td : _tds) {
    auto p = Label::getHardwareByName<middleware::ForceSensor>(td);
    if (nullptr == p) {
      LOG_ERROR << "Get the TD sensor fail!";
      continue;
    }
    td_handles_.push_back(p);
    LOG_INFO << "Get TD: " << td;
  }

  MiiString imu_label;
  if (cfg->get_value(getLabel(), "imu", imu_label)) {
    imu_handle_ = Label::getHardwareByName<middleware::ImuSensor>(imu_label);
    if (nullptr == imu_handle_)
      LOG_ERROR << "The named '" << imu_label << "' qr.res.imu sensor does not exist!";
  }

  imu_ang_vel_ = imu_handle_->angular_velocity_const_pointer();

//  joint_state_publisher_.reset( new realtime_tools::RealtimePublisher<
//    std_msgs::Float64MultiArray>(n, "/dragon/joint_commands", 10));

  // foot contact process
  foot_contact = new FootContact();
  foot_contact->setThreshold(920, 900, 760, 850);
  foot_contact->setUpperThreshold(3800);
  foot_contact->init();

  swing = new Swing();
  swing->init();

  math = new Math();
  math->init();

  gesture = new Gesture();
  gesture->init();

  for(int i=0;i<Leg_Num;i++) {
    height.push_back(-Stance_Height);
    SupportLeg.push_back(true);
  }

  __initAllofData();

  Isstand_stable = false;
  All_on_ground = false;

  timer_ = new middleware::Timer;
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
      auto diff = (leg_cmds_[l].target - leg_ifaces_[l]->joint_position_const_ref()).norm();
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
  gesture->updateImuData(imu_quat_[0], imu_quat_[1], imu_quat_[2]);
  // gesture->printImuAngle();
  foot_contact->footForceDataUpdate(
      td_handles_[0]->force_data(), td_handles_[1]->force_data(),
      td_handles_[2]->force_data(), td_handles_[3]->force_data());
}

void Walk::post_tick() {
  // std::cout << "Walk::post_tick()" << std::endl;
  command_assign(Angle_ptr);
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[leg]->legTarget(leg_cmds_[leg]);
    leg_ifaces_[leg]->move();
  }
}

/**************************************************************************
 Description: Update Angles
**************************************************************************/
void Walk::sensor_height() {
  Angle_ptr->lf.knee = *(jnt_poss_[0]);
  Angle_ptr->lf.hip  = *(jnt_poss_[1]);
  Angle_ptr->lf.pitch= *(jnt_poss_[2]);

  Angle_ptr->rf.knee = *(jnt_poss_[3]);
  Angle_ptr->rf.hip  = *(jnt_poss_[4]);
  Angle_ptr->rf.pitch= *(jnt_poss_[5]);

  Angle_ptr->lb.knee = *(jnt_poss_[6]);
  Angle_ptr->lb.hip  = *(jnt_poss_[7]);
  Angle_ptr->lb.pitch= *(jnt_poss_[8]);

  Angle_ptr->rb.knee = *(jnt_poss_[9]);
  Angle_ptr->rb.hip  = *(jnt_poss_[10]);
  Angle_ptr->rb.pitch= *(jnt_poss_[11]);
}


/**************************************************************************
 Description: initializition from robot_description
**************************************************************************/
void Walk::__initAllofData() {
  jnt_poss_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  jnt_vels_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  jnt_tors_.reserve(LegType::N_LEGS * JntType::N_JNTS);

  auto jnt_manager = middleware::JointManager::instance();
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
      auto j = jnt_manager->getJointHandle(leg, jnt);
      jnt_poss_.push_back(j->joint_position_const_pointer());
      jnt_vels_.push_back(j->joint_velocity_const_pointer());
      jnt_tors_.push_back(j->joint_torque_const_pointer());
    }
  }

  imu_ang_vel_ = imu_handle_->angular_velocity_const_pointer();
  imu_lin_acc_ = imu_handle_->linear_acceleration_const_pointer();
  imu_quat_    = imu_handle_->orientation_const_pointer();
}

void Walk::waiting() {
  LOG_WARNING << "Waiting the user operator, press any key to continue.";
  getchar();
}

void Walk::pose_init() {
  if (!is_send_init_cmds_) {
    Pos_ptr->rb.z = -Stance_Height;
    Pos_ptr->lb.z = -Stance_Height;
    Pos_ptr->rf.z = -Stance_Height;
    Pos_ptr->lf.z = -Stance_Height;

    Pos_ptr->lf.y = 0;
    Pos_ptr->rf.y = 0;
    Pos_ptr->lb.y = 0;
    Pos_ptr->rb.y = 0;

    Pos_ptr->lf.x = 0;
    Pos_ptr->rf.x = 0;
    Pos_ptr->lb.x = 0;
    Pos_ptr->rb.x = 0;
    reverse_kinematics();
  }

  is_send_init_cmds_ = true;
  PRINT_POS_VS_TARGET

}

LegType Walk::choice_next_leg(const LegType curr) {
  switch (curr) {
  case LegType::FL: return LegType::HR;
  case LegType::FR: return LegType::HL;
  case LegType::HL: return LegType::FL;
  case LegType::HR: return LegType::FR;
  default: return LegType::UNKNOWN_LEG;
  }
}

void Walk::hang_walk() {
  //LOG_WARNING << "--------------------hang walk--------------------------";
  count_loop++;
  if (count_loop > 5) {
    count_loop = 0;

    switch (internal_order_) {
      case 1:
        Isstand_stable = true;
        foot_contact->clear();
        Loop_Count = 0;
        internal_order_=2;
        break;
      case 2:
        move_cog();
        if(Loop_Count>=Stance_Num)
        {
          Loop_Count = 0;
          internal_order_=3;
          next_foot_pt();
          LOG_WARNING << "*******----case 3----*******";
        }
        break;
      case 3: //swing leg
        swing_leg(swing_leg_);
        if((Leg_On_Ground && !is_hang_walk_) || (Loop_Count>=Swing_Num && is_hang_walk_))
        // if(Loop_Count>=Swing_Num)
        {
          // std::cout<<"flow_control: swing done"<<std::endl;
          test_tmp = 0;
          Loop_Count = 0;
          for(int i=0;i<Leg_Num;i++)
          {
            support_legs_[swing_leg_] = true;
          }
          foot_contact->clear();

          internal_order_ = 2;

          swing_leg_ = choice_next_leg(swing_leg_);
          // Leg_Order = (Leg_Order>1)?Leg_Order-2:3-Leg_Order;
          std::cout<<"*******----case 4----*******"<<std::endl;
        }
        break;
      default:
        LOG_ERROR << "What fucking control!";
        break;
    } // end switch (Time_Order)
  Loop_Count++;
  } // end if(count_loop>5)
/*************************************************************************************************************/
/*******************************************Loop: 100 Hz******************************************************/
/*************************************************************************************************************/
  printf("HEIGHT: %+02.04f %+02.04f %+02.04f %+02.04f\n",
      Pos_ptr->lf.z, Pos_ptr->rf.z, Pos_ptr->lb.z, Pos_ptr->rb.z);

  //LOG_WARNING << "====================hang walk==========================";
  //printf("\n\n");
  // LOG_INFO << "Loop time used: " << timer_->dt() << " ms.";
}

void Walk::next_foot_pt() {
  switch(swing_leg_)
  {
    case LegType::FL:
      Pos_start = Pos_ptr->lf;
      Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[LF]);
      break;
    case LegType::FR:
      Pos_start = Pos_ptr->rf;
      Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[RF]);
      break;
    case LegType::HL:
      Pos_start = Pos_ptr->lb;
      Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[LB]);
      break;
    case LegType::HR:
      Pos_start = Pos_ptr->rb;
      Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[RB]);
      break;
    default:break;
  }
}

void Walk::move_cog() {
  _Position adj = {0,0,0};

  if (Loop_Count <= 1)
  {
    Cog_adj = get_CoG_adj_vec(Desired_Foot_Pos, swing_leg_);
    // std::cout<<"CoG adjust vector:"<<Cog_adj.x<<", "<<Cog_adj.y<<std::endl;
    if ((LegType::FL == swing_leg_) || (LegType::FR == swing_leg_))
      Stance_Num = 1;
    else
      Stance_Num = 50;
  }

  adj = get_stance_velocity(Cog_adj, Loop_Count);
  // std::cout<<"cog_adj h_adj_step:";


  cog_pos_assign(adj);
  reverse_kinematics();
  // std::cout<<"cog_adj after: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
}

_Position Walk::get_CoG_adj_vec(const _Position& Next_Foothold, LegType leg) {
  _Position crosspoint;
  switch (leg)
  {
    case LegType::FL:
    case LegType::HL:
      crosspoint = math->getCrossPoint(Pos_ptr->lf+gesture->getSingleShoulderPos(LF),
          Pos_ptr->rb+gesture->getSingleShoulderPos(RB),
          Pos_ptr->rf+gesture->getSingleShoulderPos(RF),
          Pos_ptr->lb+gesture->getSingleShoulderPos(LB));
      return innerTriangle(crosspoint,
          Pos_ptr->rf+gesture->getSingleShoulderPos(RF),
          Pos_ptr->rb+gesture->getSingleShoulderPos(RB));
    case LegType::FR:
    case LegType::HR:
      crosspoint = math->getCrossPoint(
          Pos_ptr->rf+gesture->getSingleShoulderPos(RF),
          Pos_ptr->lb+gesture->getSingleShoulderPos(LB),
          Pos_ptr->lf+gesture->getSingleShoulderPos(LF),
          Pos_ptr->rb+gesture->getSingleShoulderPos(RB));
      return innerTriangle(crosspoint,
          Pos_ptr->lf+gesture->getSingleShoulderPos(LF),
          Pos_ptr->lb+gesture->getSingleShoulderPos(LB));
  }
}

void Walk::swing_leg(const LegType& leg) {
  EV3 foot_vel(0,0,0),joint_vel(0,0,0),joint_pos(0,0,0);
  _Position s = {0,0,0};
  Leg_On_Ground = false;

  SupportLeg[Leg_Order1] = false;

  if(Loop_Count <= Swing_Num) {
    if(Loop_Count > Swing_Num/3*2) {
      Leg_On_Ground = foot_contact->singleFootContactStatus(leg);
    }
    if(!Leg_On_Ground) {
      s = swing->compoundCycloidPosition(Pos_start, Desired_Foot_Pos, Loop_Count, Swing_Num, Swing_Height);
      // foot_vel = swing->compoundCycloidVelocity(Pos_start, Desired_Foot_Pos, Loop_Count, Swing_Num, Swing_Height);

      switch(leg)
      {
        case LegType::FL:
          Pos_ptr->lf = Pos_start + s;
        break;
        case LegType::FR:
          Pos_ptr->rf = Pos_start + s;
        break;
        case LegType::HL:
          Pos_ptr->lb = Pos_start + s;
        break;
        case LegType::HR:
          Pos_ptr->rb = Pos_start + s;
        break;
        default:break;
      }
    }

    // cog moving
    if(Loop_Count<=Swing_Num/3*2)
    {
      Stance_Num = Swing_Num/3*2;
      s = get_stance_velocity(swing_adj_CoG, Loop_Count);
      cog_swing(s, leg);
    }
    //vel cal
    if (LegType::FL == leg) {
      joint_pos(0) = Angle_ptr->lf.pitch;
      joint_pos(1) = Angle_ptr->lf.hip;
      joint_pos(2) = Angle_ptr->lf.knee;
    } else if (LegType::FR == leg)  {
      joint_pos(0) = Angle_ptr->rf.pitch;
      joint_pos(1) = Angle_ptr->rf.hip;
      joint_pos(2) = Angle_ptr->rf.knee;
    } else if (LegType::HL == leg)  {
      joint_pos(0) = Angle_ptr->lb.pitch;
      joint_pos(1) = Angle_ptr->lb.hip;
      joint_pos(2) = Angle_ptr->lb.knee;
    } else if (LegType::HR == leg)  {
      joint_pos(0) = Angle_ptr->rb.pitch;
      joint_pos(1) = Angle_ptr->rb.hip;
      joint_pos(2) = Angle_ptr->rb.knee;
    }

    if(math->isJacobInvertible(joint_pos)) {
      LOG_WARNING << "What fucking code!";
//      commands[3*Leg_Order].has_velocity_ = true;
//      commands[3*Leg_Order+1].has_velocity_ = true;
//      commands[3*Leg_Order+2].has_velocity_ = true;
    }
//    joint_vel = math->footVelToJoint(joint_pos, foot_vel);
//    commands[3*Leg_Order].velocity_ = joint_vel(2);
//    commands[3*Leg_Order+1].velocity_ = joint_vel(1);
//    commands[3*Leg_Order+2].velocity_ = joint_vel(0);
  } else {
    Leg_On_Ground = foot_contact->singleFootContactStatus(leg);
    if(!Leg_On_Ground) {
      on_ground(leg);
      std::cout << "ON ground_control is working" << std::endl;
    }
  }
  reverse_kinematics();
}

void Walk::on_ground(const LegType& l) {
  double err = 0.1;
  switch (l)
  {
  case LegType::FL:
    Pos_ptr->lf.z = Pos_ptr->lf.z - err;
    Angle_ptr->lf = math->cal_kinematics(Pos_ptr->lf,-1);
    break;
  case LegType::FR:
    Pos_ptr->rf.z = Pos_ptr->rf.z - err;
    Angle_ptr->rf = math->cal_kinematics(Pos_ptr->rf,-1);
    break;
  case LegType::HL:
    Pos_ptr->lb.z = Pos_ptr->lb.z - err;
    Angle_ptr->lb = math->cal_kinematics(Pos_ptr->lb, 1);
    break;
  case LegType::HR:
    Pos_ptr->rb.z = Pos_ptr->rb.z - err;
    Angle_ptr->rb = math->cal_kinematics(Pos_ptr->rb, 1);
    break;
  default:
    LOG_ERROR << "What fucking LEG";
  }
}

void Walk::cog_swing(const _Position& Adj, const LegType& leg) {
  switch(leg)
  {
    case LegType::FL:
      Pos_ptr->rf = Pos_ptr->rf - Adj;
      Pos_ptr->lb = Pos_ptr->lb - Adj;
      Pos_ptr->rb = Pos_ptr->rb - Adj;
      break;
    case LegType::FR:
      Pos_ptr->lf = Pos_ptr->lf - Adj;
      Pos_ptr->lb = Pos_ptr->lb - Adj;
      Pos_ptr->rb = Pos_ptr->rb - Adj;
      break;
    case LegType::HL:
      Pos_ptr->lf = Pos_ptr->lf - Adj;
      Pos_ptr->rf = Pos_ptr->rf - Adj;
      Pos_ptr->rb = Pos_ptr->rb - Adj;
      break;
    case LegType::HR:
      Pos_ptr->lf = Pos_ptr->lf - Adj;
      Pos_ptr->rf = Pos_ptr->rf - Adj;
      Pos_ptr->lb = Pos_ptr->lb - Adj;
      break;
    default:break;
  }
}

/*************************************************************************************************************/
/*******************************************  OLD CODE  ******************************************************/
/*************************************************************************************************************/
void Walk::walk() {
/*************************************************************************************************************/
/*******************************************Loop: 100 Hz******************************************************/
/*************************************************************************************************************/
  gesture->updateImuData(imu_quat_[0],imu_quat_[1],imu_quat_[2]);
  // gesture->printImuAngle();

  foot_contact->footForceDataUpdate(td_handles_[0]->force_data(), td_handles_[1]->force_data(),
                                   td_handles_[2]->force_data(), td_handles_[3]->force_data());
  foot_contact->printForce();

  if(Isstand_stable && !is_hang_walk_)//contact_control
  {
    foot_contact->tdEventDetect();
    foot_contact->printContactStatus();
    // posture_keep();
    All_on_ground = contact_keep(foot_contact->contactStatus());
    // foot_contact->printForce();
    // std::cout<<"desired Height:"<<height[0]<<" "<<height[1]<<" "
    // <<height[2]<<" "<<height[3]<<" "<<std::endl;
    // std::cout<<"Calcued Height:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "
    // <<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
  }
/*************************************************************************************************************/
/*******************************************Loop: 20 Hz*******************************************************/
/*************************************************************************************************************/
  count_loop++;
  // if(count_loop>5 ||(Time_Order==3 && Loop_Count>Swing_Num/3*2 && foot_contact->singleFootContactStatus(Leg_Order)))
  if(count_loop>5) {
  count_loop = 0;

  switch (internal_order_)
  {
    case 0: //init gesture
      pose_init();
      flow_control1(internal_order_);
      break;
    case 1:
      if(!is_hang_walk_) {
        foot_contact->tdEventDetect();
        if(Loop_Count>3)
        {
          Isstand_stable = stand_stable(foot_contact->contactStatus());
        }
        if(Isstand_stable)
        {
          gesture->imuCalibration();
        }
      } else {
        Isstand_stable = true;
      }
      flow_control1(internal_order_);
      break;
    case 2:
      cog_adj1();
      flow_control1(internal_order_);
      break;
    case 3://swing leg
      swing_control1();
      flow_control1(internal_order_);
      break;
    default:
      LOG_ERROR << "What fucking control!";
      break;
  }
  Loop_Count++;
  }
/*************************************************************************************************************/
/*******************************************Loop: 100 Hz******************************************************/
/*************************************************************************************************************/
  command_assign(Angle_ptr);

  forward_control(1,0,0);//KP,Kv,Kd
  std::cout<<"update: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;

  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    leg_ifaces_[leg]->legTarget(leg_cmds_[leg]);
    leg_ifaces_[leg]->move();
  }
  LOG_INFO << "Loop time used: " << timer_->dt() << " ms.";
}



bool Walk::stand_stable(std::vector<bool> IsContact) {
  int phase = 1, t = 100;
  float h = 0, mean = 0, beta = 0.01;

  std::cout<<"stand stable:"<<IsContact[LF]<<" "<<IsContact[RF]<<" "<<IsContact[LB]<<" "<<IsContact[RB]<<std::endl;
  foot_contact->printForce();

  if(IsContact[LF] && IsContact[RF] && IsContact[LB] &&IsContact[RB])
  {
    phase = 2;
  }
  //phase one: make sure all legs are in touch with ground, using foot force"
  if(phase == 1)
  {
    std::cout<<"phase one: make sure all legs are in touch with ground, using foot force"<<std::endl;
    for(int i=0;i<Leg_Num;i++)
    {
      if(!IsContact[i])
      {
        height[i] -= beta;
      }
    }
  }
  //phase two: make sure overall force are balanced
  if(phase == 2)
  {
    std::cout<<"phase two: make sure overall force are balanced"<<std::endl;

    mean = (foot_contact->getForceData(LF) + foot_contact->getForceData(RF)
           +foot_contact->getForceData(LB) + foot_contact->getForceData(RB))/4.0;

    for(int i=0;i<Leg_Num;i++)
    {
      if(foot_contact->getForceData(i) < mean-t)
      {
        height[i] -= beta;
      }
      else if(foot_contact->getForceData(i) > mean+t)
      {
        height[i] += beta;
      }
      else
      {
        phase++;
      }
    }

    if(phase == 6)
    {
      phase = 3;
    }
  }
  //phase three: make sure overall height are close to desired height
  if(phase == 3)
  {
    std::cout<<"phase three: make sure overall height are close to desired height"<<std::endl;

    h = (height[LF]+height[RF]+height[LB]+height[RB])/4.0;
    // std::cout<<"stand stable: h"<<h<<" "<<fabs(Stance_Height - fabs(h))<<std::endl;
    if(fabs(Stance_Height - fabs(h))>beta)
    {
      for(int i=0;i<Leg_Num;i++)
      {
        height[i] -= Sgn(Stance_Height + h) * beta;
      }
    }
    else
    {
      if(IsContact[LF] && IsContact[RF] && IsContact[LB] &&IsContact[RB])
      {
        phase = 4;
      }
      else
      {
        phase = 2;
      }
    }
  }
  // using adjusted height to calculate angle in next loop
  Pos_ptr->lf.z = height[LF];
  Pos_ptr->rf.z = height[RF];
  Pos_ptr->lb.z = height[LB];
  Pos_ptr->rb.z = height[RB];
  reverse_kinematics();
  // std::cout<<"stable stand: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;

  //phase three: record force data
  if(phase == 4)
  {
    foot_contact->setConst(foot_contact->getForceData(LF), foot_contact->getForceData(RF),
           foot_contact->getForceData(LB), foot_contact->getForceData(RB));
    foot_contact->printConst();
    return true;
  }
  return false;
}

//after knowing which legs are all on ground, then under force limit recover height
bool Walk::contact_keep(std::vector<bool> IsContact) {
  std::cout<<"stand stable:"<<IsContact[LF]<<" "<<IsContact[RF]<<" "<<IsContact[LB]<<" "<<IsContact[RB]<<std::endl;
  // foot_contact->printForce();

  float beta = 0.05, count = 0, leg_count = 0, h = 0;

  if(SupportLeg[LF]) {
    leg_count++;
    if(IsContact[LF]==false) {
      Pos_ptr->lf.z -= beta;
    } else if(foot_contact->overDetect(LF)) {
      Pos_ptr->lf.z += beta;
    } else if(fabs(Pos_ptr->lf.z-height[LF]) > beta) {
      Pos_ptr->lf.z -= Sgn(Pos_ptr->lf.z-height[LF]) * beta;
    } else {
      count++;
    }
  }

  if(SupportLeg[RF]) {
    leg_count++;
    if(IsContact[RF]==false) {
      Pos_ptr->rf.z -= beta;
    } else if(foot_contact->overDetect(RF)) {
      Pos_ptr->rf.z += beta;
    } else if(fabs(Pos_ptr->rf.z-height[RF]) > beta) {
      Pos_ptr->rf.z -= Sgn(Pos_ptr->rf.z-height[RF]) * beta;
    } else {
      count++;
    }
  }
  if(SupportLeg[LB]) {
    leg_count++;
    if(IsContact[LB]==false) {
      Pos_ptr->lb.z -= beta;
    } else if(foot_contact->overDetect(LB)) {
      Pos_ptr->lb.z += beta;
    } else if(fabs(Pos_ptr->lb.z-height[LB]) > beta) {
      Pos_ptr->lb.z -= Sgn(Pos_ptr->lb.z-height[LB]) * beta;
    } else {
      count++;
    }
  }

  if(SupportLeg[RB]) {
    leg_count++;
    if(IsContact[RB]==false) {
      Pos_ptr->rb.z -= beta;
    } else if(foot_contact->overDetect(RB)) {
      Pos_ptr->rb.z += beta;
    } else if(fabs(Pos_ptr->rb.z-height[RB]) > beta) {
      Pos_ptr->rb.z -= Sgn(Pos_ptr->rb.z-height[RB]) * beta;
    } else {
      count++;
    }
  }

  reverse_kinematics();
  std::cout<<"contact: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;

  return ((count==leg_count) ? true:false);
}

void Walk::forward_control(float Kp,float Kv,float Kd)
{
float result = 0;

for(int i=0;i<Joint_Num;i++)
{
  ;
//  if(commands[i].has_velocity_)
//  {
//    commands[i].position_ = single_forward_control(
//      commands[i].position_, commands[i].velocity_, Kp, Kv ,Kd,
//      *(jnt_poss_[i]), *(jnt_vels_[i]));
//  }
}
}

float Walk::single_forward_control(
float d_pos, float d_vel, float Kp, float Kv, float Kd, float f_pos, float f_vel)
{
float err_pos = d_pos - f_pos;
float result = Kp*err_pos + Kd*d_vel - Kv*f_vel;
// return result;
return d_pos;
}

void Walk::flow_control1(int timeorder)
{
switch(timeorder)
{
  case 0:
  if(Loop_Count>=5)
  {
    Loop_Count = 0;
    std::cout<<"Robot has been initialized! Please input 1 to continue:";
    std::cin>>internal_order_;
    // Time_Order=1;
  }
  break;

  case 1:
  // if(Loop_Count>=30)
  if(Isstand_stable) 
  {
    foot_contact->clear();    
    Loop_Count = 0;
    // std::cout<<"Imu data calibration done!"<<std::endl;
    if(!is_hang_walk_)
    {
      std::cout<<"Robot has stand stable! Please input 2 to continue:"; 
      std::cin>>internal_order_; 
    }
    else
    {
      internal_order_=2;
    } 
  }
  break;

  case 2:
  if(Loop_Count>=Stance_Num)
  {
    Loop_Count = 0;
    // std::cout<<"flow_control: cog done"<<std::endl;
    internal_order_=3;
    // std::cout<<"Robot moving CoG! Please input 3 to continue:";
    // std::cin>>Time_Order;
    assign_next_foot1();
    std::cout<<"*******-----------------------------------case 3---------------------------------------*******"<<std::endl;

  }
  break;

  case 3:
  if((Leg_On_Ground && !is_hang_walk_) || (Loop_Count>=Swing_Num && is_hang_walk_))
  // if(Loop_Count>=Swing_Num)
  {
    // std::cout<<"flow_control: swing done"<<std::endl;
    test_tmp = 0;
    Loop_Count = 0;
    for(int i=0;i<Leg_Num;i++)
    {
      SupportLeg[Leg_Order1] = true;

    }
    foot_contact->clear();  

    internal_order_ = 2;       
    // std::cout<<"Robot swing done! Please input 2 to continue:";
    // std::cin>>Time_Order;      
    Leg_Order1 = (Leg_Order1>1)?Leg_Order1-2:3-Leg_Order1;
    std::cout<<"*******-----------------------------------case 4----------------------------------------*******"<<std::endl;
  }       
  break;

  // case 4:
  // if(All_on_ground)
  // {
  //   All_on_ground = false;
  //   Loop_Count = 0;
  //   foot_contact->clear();
  //   // std::cout<<"Robot Contact adjust done! Please input 2 to continue:";
  //   // std::cin>>Time_Order;
  //   Time_Order = 2;
  //   // std::cout<<"All_on: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
  //   std::cout<<"*******------------------------------------case 2---------------------------------------*******"<<std::endl;

  // }

  default:break;
}
}

/**************************************************************************
 Description: Ensure the foot is on ground by recursive decreasing height
 Input: lf,rf,lb,rb corresponding to 0,1,2,3
**************************************************************************/
void Walk::on_ground_control1(int legId)
{
double err = 0.1;
switch (legId)
{
  case LF:
    // Height.lf = Height.lf + err;
  Pos_ptr->lf.z = Pos_ptr->lf.z - err;
  Angle_ptr->lf = math->cal_kinematics(Pos_ptr->lf,-1);
  break;
  case RF:
    // Height.rf = Height.rf + err;
  Pos_ptr->rf.z = Pos_ptr->rf.z - err;
  Angle_ptr->rf = math->cal_kinematics(Pos_ptr->rf,-1);
  break;
  case LB:
    // Height.lb = Height.lb + err;
  Pos_ptr->lb.z = Pos_ptr->lb.z - err;
  Angle_ptr->lb = math->cal_kinematics(Pos_ptr->lb,1);
  break;
  case RB:
    // Height.rb = Height.rb + err;
  Pos_ptr->rb.z = Pos_ptr->rb.z - err;
  Angle_ptr->rb = math->cal_kinematics(Pos_ptr->rb,1);
  break;
  default:break;
}
}

/**************************************************************************
 Description: Initialization. Foot position is relative to its corresponding shoulder
**************************************************************************/

void Walk::forward_kinematics()
{
Pos_ptr->lf = math->cal_formula(Angle_ptr->lf);
Pos_ptr->rf = math->cal_formula(Angle_ptr->rf);
Pos_ptr->lb = math->cal_formula(Angle_ptr->lb);
Pos_ptr->rb = math->cal_formula(Angle_ptr->rb);
}


void Walk::reverse_kinematics() {
Angle_ptr->lf = math->cal_kinematics(Pos_ptr->lf,-1);
Angle_ptr->rf = math->cal_kinematics(Pos_ptr->rf,-1);
Angle_ptr->lb = math->cal_kinematics(Pos_ptr->lb, 1);
Angle_ptr->rb = math->cal_kinematics(Pos_ptr->rb, 1);
}

_Position Walk::innerTriangle(const _Position &A, const _Position &B, const _Position &C)
{
_Position innerheart, a_inner, b_inner, c_inner, first_cross, second_cross;
_Position cog_init(0,0,0);
float ratio = 1;
float radius = math->inscribedCircleRadius(A,B,C);

// std::cout<<"inscribedCircleRadius:"<<radius<<std::endl;

innerheart = math->getInnerHeart(A, B, C);
// std::cout<<"Inner heart:"<<innerheart.x<<","<<innerheart.y<<std::endl;

if(radius>cogThreshold)
{
  ratio = (radius-cogThreshold)/radius;

  a_inner = math->formulaLineSection(A, innerheart, ratio);
  b_inner = math->formulaLineSection(B, innerheart, ratio);
  c_inner = math->formulaLineSection(C, innerheart, ratio);

  first_cross = math->getCrossPoint(a_inner,b_inner,cog_init,innerheart);
  second_cross = math->getCrossPoint(a_inner,c_inner,cog_init,innerheart);

  // std::cout<<"Ratio:"<<ratio<<std::endl;
  // std::cout<<"A:"<<A.x<<","<<A.y<<" B:"<<B.x<<","<<B.y<<" C:"<<C.x<<","<<C.y<<std::endl;
  // std::cout<<"a:"<<a_inner.x<<","<<a_inner.y<<" b:"<<b_inner.x<<","<<b_inner.y<<" c:"<<c_inner.x<<","<<c_inner.y<<std::endl;

  // std::cout<<"first_cross:"<<first_cross.x<<","<<first_cross.y<<std::endl;
  // std::cout<<"second_cross:"<<second_cross.x<<","<<second_cross.y<<std::endl;

  if(first_cross.x<=max(a_inner.x, b_inner.x) && first_cross.x>=min(a_inner.x, b_inner.x))
  {
    // std::cout<<"first_cross true"<<std::endl;
    swing_adj_CoG = (b_inner.x>c_inner.x) ? b_inner - first_cross : c_inner - first_cross;
    swing_adj_CoG.x = swing_adj_CoG.x/2.0;
    swing_adj_CoG.y = swing_adj_CoG.y/2.0;
    // std::cout<<"swing_adj_CoG: "<<swing_adj_CoG.x <<" "<<swing_adj_CoG.y<<std::endl;
    return first_cross;
  }
  if(second_cross.x<=max(a_inner.x, c_inner.x) && second_cross.x>=min(a_inner.x, c_inner.x))
  {
    // std::cout<<"second_cross true"<<std::endl;
    swing_adj_CoG = (b_inner.x>c_inner.x) ? b_inner - second_cross : c_inner - second_cross;
    swing_adj_CoG.x = swing_adj_CoG.x/2.0;
    swing_adj_CoG.y = swing_adj_CoG.y/2.0;
    // std::cout<<"swing_adj_CoG: "<<swing_adj_CoG.x <<" "<<swing_adj_CoG.y<<std::endl;
    return second_cross;
  }
}
else
{
  return innerheart;
}
}
/**************************************************************************
 Author: WangShanren
 Date: 2017.2.22
 Description: calculating CoG position and CoG adjust vector,using next foothold,while 1 means to right side
 Input: four feet(end effector) position(x,y,z),Swing leg order and next Swing leg position
 Output: CoG adjust vector
**************************************************************************/
_Position Walk::get_CoG_adj_vec1(const _Position &Next_Foothold, unsigned int Swing_Order)
{
_Position crosspoint;
switch (Swing_Order)
{
  case LF:
  case LB:
  crosspoint = math->getCrossPoint(Pos_ptr->lf+gesture->getSingleShoulderPos(LF), Pos_ptr->rb+gesture->getSingleShoulderPos(RB),Pos_ptr->rf+gesture->getSingleShoulderPos(RF), Pos_ptr->lb+gesture->getSingleShoulderPos(LB));
  return innerTriangle(crosspoint, Pos_ptr->rf+gesture->getSingleShoulderPos(RF), Pos_ptr->rb+gesture->getSingleShoulderPos(RB));
  case RF:
  case RB:
  crosspoint = math->getCrossPoint(Pos_ptr->rf+gesture->getSingleShoulderPos(RF), Pos_ptr->lb+gesture->getSingleShoulderPos(LB),
  Pos_ptr->lf+gesture->getSingleShoulderPos(LF), Pos_ptr->rb+gesture->getSingleShoulderPos(RB));
  return innerTriangle(crosspoint, Pos_ptr->lf+gesture->getSingleShoulderPos(LF), Pos_ptr->lb+gesture->getSingleShoulderPos(LB));
}
}

void Walk::assign_next_foot1()
{
switch(Leg_Order1)
{
  case LF:
  Pos_start = Pos_ptr->lf;
  Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[LF]);
  break;
  case RF:
  Pos_start = Pos_ptr->rf;
  Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[RF]);
  break;
  case LB:
  Pos_start = Pos_ptr->lb;
  Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[LB]);
  break;
  case RB:
  Pos_start = Pos_ptr->rb;
  Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[RB]);
  break;
  default:break;
}
}

void Walk::cog_adj1()
{
// Switch = 1;
_Position adj = {0,0,0};

if(Loop_Count<=1)
{
  Cog_adj = get_CoG_adj_vec1(Desired_Foot_Pos, Leg_Order1);
  // std::cout<<"CoG adjust vector:"<<Cog_adj.x<<", "<<Cog_adj.y<<std::endl;
  Stance_Num = (Leg_Order1<2) ? 1:50;
}

adj = get_stance_velocity(Cog_adj, Loop_Count);
// std::cout<<"cog_adj h_adj_step:";


cog_pos_assign(adj);
reverse_kinematics();
// std::cout<<"cog_adj after: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;

}

void Walk::swing_control1() {
  EV3 foot_vel(0,0,0),joint_vel(0,0,0),joint_pos(0,0,0);
  _Position s = {0,0,0};
  Leg_On_Ground = false;

  SupportLeg[Leg_Order1] = false;

  if(Loop_Count <= Swing_Num) {
    if(Loop_Count > Swing_Num/3*2) {
      Leg_On_Ground = foot_contact->singleFootContactStatus1(Leg_Order1);
    }
    if(!Leg_On_Ground) {
      s = swing->compoundCycloidPosition(Pos_start, Desired_Foot_Pos, Loop_Count, Swing_Num, Swing_Height);
      // foot_vel = swing->compoundCycloidVelocity(Pos_start, Desired_Foot_Pos, Loop_Count, Swing_Num, Swing_Height);

      switch(Leg_Order1)
      {
        case LF:
          Pos_ptr->lf = Pos_start + s;
        break;
        case RF:
          Pos_ptr->rf = Pos_start + s;
        break;
        case LB:
          Pos_ptr->lb = Pos_start + s;
        break;
        case RB:
          Pos_ptr->rb = Pos_start + s;
        break;
        default:break;
      }
    }
    //cog moving
    if(Loop_Count<=Swing_Num/3*2)
    {
      Stance_Num = Swing_Num/3*2;
      s = get_stance_velocity(swing_adj_CoG, Loop_Count);
      cog_swing_assign1(s, Leg_Order1);
    }
    //vel cal
    if(Leg_Order1==LF) {
      joint_pos(0) = Angle_ptr->lf.pitch;
      joint_pos(1) = Angle_ptr->lf.hip;
      joint_pos(2) = Angle_ptr->lf.knee;
    } else if(Leg_Order1==RF) {
      joint_pos(0) = Angle_ptr->rf.pitch;
      joint_pos(1) = Angle_ptr->rf.hip;
      joint_pos(2) = Angle_ptr->rf.knee;
    } else if(Leg_Order1==LB) {
      joint_pos(0) = Angle_ptr->lb.pitch;
      joint_pos(1) = Angle_ptr->lb.hip;
      joint_pos(2) = Angle_ptr->lb.knee;
    } else if(Leg_Order1==RB) {
      joint_pos(0) = Angle_ptr->rb.pitch;
      joint_pos(1) = Angle_ptr->rb.hip;
      joint_pos(2) = Angle_ptr->rb.knee;
    }

    if(math->isJacobInvertible(joint_pos)) {
      LOG_WARNING << "What fucking code!";
//      commands[3*Leg_Order].has_velocity_ = true;
//      commands[3*Leg_Order+1].has_velocity_ = true;
//      commands[3*Leg_Order+2].has_velocity_ = true;
    }
//    joint_vel = math->footVelToJoint(joint_pos, foot_vel);
//    commands[3*Leg_Order].velocity_ = joint_vel(2);
//    commands[3*Leg_Order+1].velocity_ = joint_vel(1);
//    commands[3*Leg_Order+2].velocity_ = joint_vel(0);
  } else {
    Leg_On_Ground = foot_contact->singleFootContactStatus1(Leg_Order1);
    if(!Leg_On_Ground) {
      on_ground_control1(Leg_Order1);
      std::cout << "ON ground_control is working" << std::endl;
    }
  }
  reverse_kinematics();
}

void Walk::cog_pos_assign(_Position Adj) {
  Pos_ptr->lf = Pos_ptr->lf - Adj;
  Pos_ptr->rf = Pos_ptr->rf - Adj;
  Pos_ptr->lb = Pos_ptr->lb - Adj;
  Pos_ptr->rb = Pos_ptr->rb - Adj;
}

void Walk::cog_swing_assign1(_Position Adj, int legId) {
  switch(Leg_Order1) {
    case LF:
      Pos_ptr->rf = Pos_ptr->rf - Adj;
      Pos_ptr->lb = Pos_ptr->lb - Adj;
      Pos_ptr->rb = Pos_ptr->rb - Adj;
    break;
    case RF:
      Pos_ptr->lf = Pos_ptr->lf - Adj;
      Pos_ptr->lb = Pos_ptr->lb - Adj;
      Pos_ptr->rb = Pos_ptr->rb - Adj;
    break;
    case LB:
      Pos_ptr->lf = Pos_ptr->lf - Adj;
      Pos_ptr->rf = Pos_ptr->rf - Adj;
      Pos_ptr->rb = Pos_ptr->rb - Adj;
    break;
    case RB:
      Pos_ptr->lf = Pos_ptr->lf - Adj;
      Pos_ptr->rf = Pos_ptr->rf - Adj;
      Pos_ptr->lb = Pos_ptr->lb - Adj;
    break;
    default:  break;
  }
}

/**************************************************************************
 Author: WangShanren
 Date: 2017.2.24
 Description: after design the exact trajectory, providing velocity while move the body
 Input: CoG adjust Vector and Loop(1~)
 Output: realtime velocity
 Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
 X-axis:
 Vel(t):30 * XD * t^4 / Stance_Num^5 - 60 * XD * t^3 / Stance_Num^4 + 30 * XD * t^2 / Stance_Num^3
 Y-axis:
 Vel(t):30 * YD * t^4 / Stance_Num^5 - 60 * YD * t^3 / Stance_Num^4 + 30 * YD * t^2 / Stance_Num^3
 Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/
_Position Walk::get_stance_velocity(_Position Adj_vec, unsigned int Loop) {
  if(Loop>Stance_Num) {
    std::cout<<"Loop:"<<Loop<<" Stance_Num:"<<Stance_Num<<std::endl;
    LOG_ERROR << ("Time Order wrong while stance");
  }
  _Position stance_vel = {0,0,0};
  stance_vel.x = 30 * Adj_vec.x * pow(Loop,4) / pow(Stance_Num,5)
      - 60 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,4)
      + 30 * Adj_vec.x * pow(Loop,2) / pow(Stance_Num,3);
  stance_vel.y = 30 * Adj_vec.y * pow(Loop,4) / pow(Stance_Num,5)
      - 60 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,4)
      + 30 * Adj_vec.y * pow(Loop,2) / pow(Stance_Num,3);
  return stance_vel;
}

void Walk::command_assign(Angle_Ptr Angle) {
  leg_cmds_[LegType::FL].target(JntType::YAW)  = Angle->lf.pitch;
  leg_cmds_[LegType::FL].target(JntType::HIP)  = Angle->lf.hip;
  leg_cmds_[LegType::FL].target(JntType::KNEE) = Angle->lf.knee;

  leg_cmds_[LegType::FR].target(JntType::YAW)  = Angle->rf.pitch;
  leg_cmds_[LegType::FR].target(JntType::HIP)  = Angle->rf.hip;
  leg_cmds_[LegType::FR].target(JntType::KNEE) = Angle->rf.knee;

  leg_cmds_[LegType::HL].target(JntType::YAW)  = Angle->lb.pitch;
  leg_cmds_[LegType::HL].target(JntType::HIP)  = Angle->lb.hip;
  leg_cmds_[LegType::HL].target(JntType::KNEE) = Angle->lb.knee;

  leg_cmds_[LegType::HR].target(JntType::YAW)  = Angle->rb.pitch;
  leg_cmds_[LegType::HR].target(JntType::HIP)  = Angle->rb.hip;
  leg_cmds_[LegType::HR].target(JntType::KNEE) = Angle->rb.knee;
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

void __print_command(const LegTarget* leg_cmds_) {
  printf("_________________________________\n");
  printf("LEG -|   YAW  |   HIP  |  KNEE  |\n");
  printf("FL  -| %+01.04f| %+01.04f| %+01.04f|\n",
      leg_cmds_[LegType::FL].target(JntType::YAW),
      leg_cmds_[LegType::FL].target(JntType::HIP),
      leg_cmds_[LegType::FL].target(JntType::KNEE));

  printf("FR  -| %+01.04f| %+01.04f| %+01.04f|\n",
      leg_cmds_[LegType::FR].target(JntType::YAW),
      leg_cmds_[LegType::FR].target(JntType::HIP),
      leg_cmds_[LegType::FR].target(JntType::KNEE));

  printf("HL  -| %+01.04f| %+01.04f| %+01.04f|\n",
      leg_cmds_[LegType::HL].target(JntType::YAW),
      leg_cmds_[LegType::HL].target(JntType::HIP),
      leg_cmds_[LegType::HL].target(JntType::KNEE));

  printf("HR  -| %+01.04f| %+01.04f| %+01.04f|\n",
      leg_cmds_[LegType::HR].target(JntType::YAW),
      leg_cmds_[LegType::HR].target(JntType::HIP),
      leg_cmds_[LegType::HR].target(JntType::KNEE));
  printf("---------------------------------\n");
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::Walk, Label)
