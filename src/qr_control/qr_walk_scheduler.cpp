#include <qr_control/qr_walk_scheduler.h>

namespace qr_control {

QrWalkScheduler::QrWalkScheduler() :
    LEG_NUM(0), JNT_NUM(0),
    UPDATE_FREQ(2),
    loop_count_(0), ctrl_mode_(CTRL_STATE::MOVE_TO_INIT),
    leg_order_(1) {
  google::InitGoogleLogging("QrWalkScheduler");
  // google::SetLogDestination(google::GLOG_INFO, "/path/to/log/INFO_");
  // google::LogMessage::Init();
  FLAGS_colorlogtostderr = true;
  google::FlushLogFiles(google::GLOG_INFO);

  joint_handles_.reserve(MAX_LEG_NUM);
  fk_solvers_.reserve(MAX_LEG_NUM);
  ik_solvers_.reserve(MAX_LEG_NUM);
  leg_chains_.reserve(MAX_LEG_NUM);
}

QrWalkScheduler::~QrWalkScheduler() {
}

// for debug
void QrWalkScheduler::printJointPositions() {
  LOG_WARNING << "========================";
  LOG_WARNING << "--------POSITION--------";
  LOG_INFO << "L-B:";
  LOG_INFO << "hip: " << joint_current_positions_->lb.hip;
  LOG_INFO << "knee: "<< joint_current_positions_->lb.knee;
  LOG_INFO << "yaw: " << joint_current_positions_->lb.pitch;
/*  LOG_INFO << "foot: (" << foots_pos_->lb.x
      << ", " << foots_pos_->lb.y
      << ", " << foots_pos_->lb.z << ")";*/
  LOG_INFO << "L-F:";
  LOG_INFO << "hip: " << joint_current_positions_->lf.hip;
  LOG_INFO << "knee: "<< joint_current_positions_->lf.knee;
  LOG_INFO << "yaw: " << joint_current_positions_->lf.pitch;
/*  LOG_INFO << "foot: (" << foots_pos_->lf.x
      << ", " << foots_pos_->lf.y
      << ", " << foots_pos_->lf.z << ")";*/
  LOG_INFO << "R-B:";
  LOG_INFO << "hip: " << joint_current_positions_->rb.hip;
  LOG_INFO << "knee: "<< joint_current_positions_->rb.knee;
  LOG_INFO << "yaw: " << joint_current_positions_->rb.pitch;
  /*LOG_INFO << "foot: (" << foots_pos_->rb.x
      << ", " << foots_pos_->rb.y
      << ", " << foots_pos_->rb.z << ")";*/
  LOG_INFO << "R-F:";
  LOG_INFO << "hip: " << joint_current_positions_->rf.hip;
  LOG_INFO << "knee: "<< joint_current_positions_->rf.knee;
  LOG_INFO << "yaw: " << joint_current_positions_->rf.pitch;
  /*LOG_INFO << "foot: (" << foots_pos_->rf.x
      << ", " << foots_pos_->rf.y
      << ", " << foots_pos_->rf.z << ")";*/
  LOG_WARNING << "========================";
}

void QrWalkScheduler::readJointEncoder(Eigen::VectorXd& angles) {
  if (angles.size() != JNT_NUM) angles.resize(JNT_NUM);

  for (int i = 0; i < JNT_NUM; ++i) {
    angles[i] = joint_handles_[i].getPosition();
  }
}


/**************************************************************************
   Description: initialize joints from robot_description
**************************************************************************/
bool QrWalkScheduler::init(PositionJointInterface* robot, ros::NodeHandle& n) {
  bool debug = true;
  ros::param::get("~debug", debug);
  if (debug) {
    google::SetStderrLogging(google::GLOG_INFO);
  } else {
    google::SetStderrLogging(google::GLOG_WARNING);
  }

  if (!robot_model_.initParam("robot_description")) {
    LOG_ERROR << "Failed to parse urdf file";
    return false;
  }

  KDL::Tree temp_kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(robot_model_, temp_kdl_tree)){
    LOG_ERROR << "Could not convert urdf into kdl tree";
    return false;
  }

  // Init fk and ik slover
  LEG_NUM = 0;
  std::string param_name;
  while(true) {
    param_name = "root_" + std::to_string(LEG_NUM);
    std::string root = "";
    if (!ros::param::get(param_name, root)) {
      break;
    }
    param_name = "tip_" + std::to_string(LEG_NUM);
    std::string tip  = "";
    if (!ros::param::get(param_name, tip)) {
      break;
    }
    // KDL chains for the end-effectors.
    KDL::Chain temp_chain;
    if (!temp_kdl_tree.getChain(root, tip, temp_chain)){
      LOG_ERROR << "Controller could not use the chain from '"
        << root << "' to '" << tip << "'";
      return false;
    }
    leg_chains_.push_back(temp_chain);
    // Pose solvers.
    boost::shared_ptr<KDL::ChainFkSolverPos> fk_sp;
    fk_sp.reset(new KDL::ChainFkSolverPos_recursive(leg_chains_[LEG_NUM]));
    fk_solvers_.push_back(fk_sp);

    boost::shared_ptr<KDL::ChainIkSolverPos> ik_sp;
    ik_sp.reset(new KDL::ChainIkSolverPos_LMA(leg_chains_[LEG_NUM]));
    ik_solvers_.push_back(ik_sp);
    ++LEG_NUM;

    JNT_NUM_OF_EVERY_LEG.push_back(temp_chain.getNrOfJoints());
  }
  eef_poses_.resize(LEG_NUM);
  eef_solves_.resize(LEG_NUM);

  JNT_NUM = 0;
  while(true) {
    std::string joint_name;
    std::string param_name = "joint_" + std::to_string(JNT_NUM);
    if (ros::param::get(param_name.c_str(), joint_name)) {
      joint_names_.push_back(joint_name);
      joint_handles_.push_back(robot->getHandle(joint_name));
    } else {
      break;
    }
    JNT_NUM++;
  }
  int joint_num = 0;
  for (auto itr : JNT_NUM_OF_EVERY_LEG) {
    joint_num += itr;
  }
  if (joint_num != JNT_NUM) {
    LOG_ERROR << "Something was wrong about the number of joint";
  }

  last_X_pos_.resize(JNT_NUM);
  init_X_pos_.resize(JNT_NUM);
  eef_poses_.resize(LEG_NUM);
  last_X_pos_.fill(0.0);
  init_X_pos_.fill(0.0);

  step_duration_ = ros::Duration(UPDATE_FREQ);
  return true;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: set model initial state
**************************************************************************/
void QrWalkScheduler::starting(const ros::Time& time) {
  LOG_INFO << "Getting the current joint position";
  joint_current_positions_->lb.pitch= joint_handles_[0].getPosition();
  joint_current_positions_->lb.hip  = joint_handles_[1].getPosition();
  joint_current_positions_->lb.knee = joint_handles_[2].getPosition();

  joint_current_positions_->lf.pitch= joint_handles_[3].getPosition();
  joint_current_positions_->lf.hip  = joint_handles_[4].getPosition();
  joint_current_positions_->lf.knee = joint_handles_[5].getPosition();

  joint_current_positions_->rb.pitch= joint_handles_[6].getPosition();
  joint_current_positions_->rb.hip  = joint_handles_[7].getPosition();
  joint_current_positions_->rb.knee = joint_handles_[8].getPosition();

  joint_current_positions_->rf.pitch= joint_handles_[9].getPosition();
  joint_current_positions_->rf.hip  = joint_handles_[10].getPosition();
  joint_current_positions_->rf.knee = joint_handles_[11].getPosition();

  // printJointPositions();

  fk();
  // std::cout<<"starting"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<std::endl;

  init_pos_[0] = foots_pos_->lf.z;
  init_pos_[1] = foots_pos_->rf.z;
  init_pos_[2] = foots_pos_->lb.z;
  init_pos_[3] = foots_pos_->rb.z;
  init_pos_[4] = foots_pos_->lf.y;
  init_pos_[5] = foots_pos_->rf.y;
  init_pos_[6] = foots_pos_->lb.y;
  init_pos_[7] = foots_pos_->rb.y;
  init_pos_[8] = foots_pos_->lf.x;
  init_pos_[9] = foots_pos_->rf.x;
  init_pos_[10]= foots_pos_->lb.x;
  init_pos_[11]= foots_pos_->rb.x;
  // std::cout<<"Angle_ptr->lf:"<<Angle_ptr->lf.knee<<std::endl;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: design state meachine: Adjust CoG <---> Switch Swing Leg
**************************************************************************/
void QrWalkScheduler::update(const ros::Time& time, const ros::Duration& period) {
  tmp_duration_ = time - last_update_time_;
  if (period < (step_duration_ - tmp_duration_))
    ;// return;
  else {
    last_update_time_ = time;
    fk(eef_poses_);
    ik(eef_poses_, eef_solves_);
  }

  switch (ctrl_mode_) {
  case CTRL_STATE::MOVE_TO_INIT: // init height
  {
    // LOG_WARNING << "Move to Init Pose";
    goInitPose();
    break;
  }
  case CTRL_STATE::CHOOSE_WHICH_LEG: // assign next foot holder value
  {
    // LOG_WARNING << "Calculate The Next Foot Point";
    getNextFoothold();
    break;
  }
  case CTRL_STATE::ADJUST_COG: // adjust CoG
  {
    // LOG_WARNING << "Adjust the CoG Position";
    cog_adj();
    break;
  }
  case CTRL_STATE::STEP_FORWARD: // swing leg
  {
    // LOG_WARNING << "Move The Leg";
    swing_control();
    break;
  }
  default: break;
  }

  ++loop_count_;
  vec_assign(joint_current_positions_, joint_cmds_);
  for(int i=0; i < JNT_NUM; i++) {
    joint_handles_[i].setCommand(joint_cmds_[i]);
  }

}

void QrWalkScheduler::goInitPose() {
  // float adj = fabs(L0 + L1 + L2 - Height);//TODO
  if(loop_count_ > Init_Num) {
    // TODO
    loop_count_ = 0;
    ctrl_mode_ = CTRL_STATE::CHOOSE_WHICH_LEG;
    // std::cout<<"Init completd input 1 to continue"<<std::endl;
    // std::cin>>ctrl_mode_;
    return;
  }

  foots_pos_->lf.z = init_pos_[0] + get_adj_pos(-HEIGHT-init_pos_[0], loop_count_, Init_Num);
  foots_pos_->rf.z = init_pos_[1] + get_adj_pos(-HEIGHT-init_pos_[1], loop_count_, Init_Num);
  foots_pos_->lb.z = init_pos_[2] + get_adj_pos(-HEIGHT-init_pos_[2], loop_count_, Init_Num);
  foots_pos_->rb.z = init_pos_[3] + get_adj_pos(-HEIGHT-init_pos_[3], loop_count_, Init_Num);

  foots_pos_->lf.y = init_pos_[4] + get_adj_pos(BODY_WIDTH-init_pos_[4], loop_count_, Init_Num);
  foots_pos_->rf.y = init_pos_[5] + get_adj_pos(-BODY_WIDTH-init_pos_[5], loop_count_, Init_Num);
  foots_pos_->lb.y = init_pos_[6] + get_adj_pos(BODY_WIDTH-init_pos_[6], loop_count_, Init_Num);
  foots_pos_->rb.y = init_pos_[7] + get_adj_pos(-BODY_WIDTH-init_pos_[7], loop_count_, Init_Num);

  foots_pos_->lf.x = init_pos_[8] + get_adj_pos(BODY_LENGTH-init_pos_[8], loop_count_, Init_Num);
  foots_pos_->rf.x = init_pos_[9] + get_adj_pos(BODY_LENGTH-init_pos_[9], loop_count_, Init_Num);
  foots_pos_->lb.x = init_pos_[10]+ get_adj_pos(-BODY_LENGTH-init_pos_[10], loop_count_, Init_Num);
  foots_pos_->rb.x = init_pos_[11]+ get_adj_pos(-BODY_LENGTH-init_pos_[11], loop_count_, Init_Num);

  ik();
}

float QrWalkScheduler::get_adj_pos(float Adj, int t, int T) {
  float pos = 6 * Adj * pow(t,5) / pow(T,5)
      - 15 * Adj * pow(t,4) / pow(T,4) + 10 * Adj * pow(t,3) / pow(T,3);
  return pos;
}

void QrWalkScheduler::getNextFoothold() {
  switch(leg_order_) { // choose swing foot
  case 1:
  {
    next_foot_pos_ = struct_assign(BODY_LENGTH - FOOT_STEP, BODY_WIDTH, -HEIGHT);
    break;
  }
  case 2:
  {
    next_foot_pos_ = struct_assign(BODY_LENGTH - FOOT_STEP, -BODY_WIDTH, -HEIGHT);
    break;
  }
  case 3:
  {
    next_foot_pos_ = struct_assign(-BODY_LENGTH - FOOT_STEP, BODY_WIDTH, -HEIGHT);
    break;
  }
  case 4:
  {
    next_foot_pos_ = struct_assign(-BODY_LENGTH - FOOT_STEP, -BODY_WIDTH, -HEIGHT);
    break;
  }
  default:
    break;
  }

  loop_count_ = 0;
  ctrl_mode_ = CTRL_STATE::ADJUST_COG;
}
void QrWalkScheduler::cog_adj() {
  __Position adj = {0,0,0};

  if (loop_count_ <= COG_COUNT) {
    if(1 == loop_count_) {
      Cog_adj = get_CoG_adj_vec(next_foot_pos_, leg_order_); // 1 means left
    }
    adj = get_stance_velocity(Cog_adj, loop_count_);
    cog_pos_assign(adj);
    ik();
  } else {
    loop_count_ = 0;
    ctrl_mode_ = CTRL_STATE::STEP_FORWARD;

    switch(leg_order_) {
    case 1:
    {
      Pos_start = struct_copy(foots_pos_->lf);
      break;
    }
    case 2:
    {
      Pos_start = struct_copy(foots_pos_->rf);
      break;
    }
    case 3:
    {
      Pos_start = struct_copy(foots_pos_->lb);
      break;
    }
    case 4:
    {
      Pos_start = struct_copy(foots_pos_->rb);
      break;
    }
    default:
      break;
    }
  }
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after knowing the exact distance to move CoG, design proper trajectory
   Input: CoG adjust Vector and Loop(1~)
   Output: realtime position
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
   X-axis:
   Pos(t):6 * XD * t^5 / Stance_Num^5 - 15 * XD * t^4 / Stance_Num^4 + 10 * XD * t^3 / Stance_Num^3
   Y-axis:
   Pos(t):6 * YD * t^5 / Stance_Num^5 - 15 * YD * t^4 / Stance_Num^4 + 10 * YD * t^3 / Stance_Num^3
   Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: initialize body pose at first time
   Input: position and Angle_ptr pointer
   Output: none
**************************************************************************/
void QrWalkScheduler::swing_control() {
  __Position s = {0,0,0};

  if (loop_count_ <= Swing_Num) {
    s = get_eclipse_pos(Pos_start, next_foot_pos_, loop_count_);
    switch(leg_order_) {
    case 1: // swing left front leg
      foots_pos_->lf = struct_assign(Pos_start.x + s.x,Pos_start.y + s.y,Pos_start.z + s.z);
      break;
    case 2: // swing right front leg
      foots_pos_->rf = struct_assign(Pos_start.x + s.x,Pos_start.y + s.y,Pos_start.z + s.z);
      break;
    case 3: // swing left back leg
      foots_pos_->lb = struct_assign(Pos_start.x + s.x,Pos_start.y + s.y,Pos_start.z + s.z);
      break;
    case 4: // swing right back leg
      foots_pos_->rb = struct_assign(Pos_start.x + s.x,Pos_start.y + s.y,Pos_start.z + s.z);
      break;
    default: break;
    }
    ik();
  } else {
    loop_count_ = 0;
    ctrl_mode_ = CTRL_STATE::CHOOSE_WHICH_LEG;
    leg_order_  = ((leg_order_ + 2) > 4) ? (5 - leg_order_) : (leg_order_ + 2);
  }
}

__Position QrWalkScheduler::get_eclipse_pos(__Position Start_point, __Position End_point,int Loop)
{
        __Position swing_foot = {0,0,0};
        int sgn = Sgn( End_point.x - Start_point.x);
        float angle = PI - PI * Loop/Swing_Num;
        float centre = (Start_point.x + End_point.x)/2;
        float a =  fabs(Start_point.x - End_point.x)/2;
        float b = Swing_Height;
        swing_foot.x = sgn*(a + a*cos(angle));
        swing_foot.z = b*sin(angle);

        sgn = Sgn( End_point.y - Start_point.y);
        a =  fabs(Start_point.y - End_point.y)/2;

        swing_foot.y = sgn*(a + a*cos(angle));

        return swing_foot;
}

void QrWalkScheduler::cog_pos_assign(__Position Adj) {
  foots_pos_->lf = struct_assign(foots_pos_->lf.x - Adj.x, foots_pos_->lf.y - Adj.y, foots_pos_->lf.z);
  foots_pos_->rf = struct_assign(foots_pos_->rf.x - Adj.x, foots_pos_->rf.y - Adj.y, foots_pos_->rf.z);
  foots_pos_->lb = struct_assign(foots_pos_->lb.x - Adj.x, foots_pos_->lb.y - Adj.y, foots_pos_->lb.z);
  foots_pos_->rb = struct_assign(foots_pos_->rb.x - Adj.x, foots_pos_->rb.y - Adj.y, foots_pos_->rb.z);
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.21
   Description: calculating forward kinematics for quadruped robot(3 DOF) using D-H methods
   Formula:
   Px = Xrf + L1 * S1 + L2 * S12;
   Py = Yrf + L0 * S0 + L1 * S0 * C1 + L2 * S0 * C12;
   Pz = Zrf - Lo * C0 - L1 * C0 * C1 - L2 * C0 * C12;
**************************************************************************/
__Position QrWalkScheduler::cal_formula(__Angle_Leg A,int L,int W) {
  __Position p;
  p.x = L + L1 * sin(A.hip)   + L2 * sin(A.hip + A.knee);
  p.y = W + L0 * sin(A.pitch) + L1 * sin(A.pitch) * cos(joint_current_positions_->lf.hip)
      + L2 * sin(A.pitch) * cos(joint_current_positions_->lf.hip + A.knee);
  p.z = -L0 * cos(A.pitch) - L1 * cos(A.pitch) * cos(A.hip) - L2 * cos(A.pitch) * cos(A.hip + A.knee);
  return p;
}

void QrWalkScheduler::fk(std::vector<KDL::Frame>& tips) {
  readJointEncoder(last_X_pos_);

  for (int leg = 0; leg < LEG_NUM; ++leg) {
    if (JNT_NUM_OF_EVERY_LEG[leg] != temp_joint_angles_.data.size())
      temp_joint_angles_.resize(JNT_NUM_OF_EVERY_LEG[leg]);

    int offset = 0;
    for (int i = 0; i < leg; ++i)
      offset += JNT_NUM_OF_EVERY_LEG[i];

    for (int jnt = 0; jnt < JNT_NUM_OF_EVERY_LEG[leg]; ++jnt) {
      temp_joint_angles_(jnt) = last_X_pos_(offset + jnt);
    }

    fk_solvers_[leg]->JntToCart(temp_joint_angles_, tips[leg]);
  }

  // for debug
  std::stringstream ss;
  ss << "\nTIP_POSEs\n";
  ss << "========================F K========================\n";
  for (int leg = 0; leg < LEG_NUM; ++leg) {
    ss << "leg_" << leg << ":\t";
    for (int coor = 0; coor < 3; ++coor) {
      ss << tips[leg].p(coor) << "\t";
    }
  }
  ss << "\n---------------------------------------------------\n";
  double x, y, z, w;
  for (int leg = 0; leg < LEG_NUM; ++leg) {
    ss << "leg_" << leg << ":\t";
    tips[leg].M.GetQuaternion(x, y, z, w);
    ss << x << "\t" << y << "\t" << z << "\t" << w << "\n";
  }
  ss << "===================================================";
  LOG_WARNING << ss.str();
}

void QrWalkScheduler::fk() {
  foots_pos_->lf = cal_formula(joint_current_positions_->lf, BODY_LENGTH, BODY_WIDTH);
  foots_pos_->rf = cal_formula(joint_current_positions_->rf, BODY_LENGTH,-BODY_WIDTH);
  foots_pos_->lb = cal_formula(joint_current_positions_->lb,-BODY_LENGTH, BODY_WIDTH);
  foots_pos_->rb = cal_formula(joint_current_positions_->rb,-BODY_LENGTH,-BODY_WIDTH);

  /*LOG_WARNING << "lf:\t" << foots_pos_->lf.x << " " << foots_pos_->lf.y
        << " " << foots_pos_->lf.z;
  LOG_WARNING << "rf:\t" << foots_pos_->rf.x << " " << foots_pos_->rf.y
        << " " << foots_pos_->rf.z;
  LOG_WARNING << "lb:\t" << foots_pos_->lb.x << " " << foots_pos_->lb.y
        << " " << foots_pos_->lb.z;
  LOG_WARNING << "rb:\t" << foots_pos_->rb.x << " " << foots_pos_->rb.y
        << " " << foots_pos_->rb.z;*/
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.21
   Description: calculating reverse kinematics for quadruped robot(3 DOF)
   Formula:
   Theta_0 = atan((Yrf - Py) / (Pz - Zrf));
   Theta_1 = 2 * atan((Epsilon + sqrt(Epsilon^2 - Phi * (L2 * S2 - Delte))) / Phi);
   Theta_2 = sgn * acos((Delte^2 + Epsilon^2 - L1^2 - L2^2) / 2 / L1 / L2);
   Meantime, Delte = Px - Xrf; Phi = Delte + L2 * S2; Epsilon = L0 + Pz * C0 - Zrf * C0 - Py * S0 + Yrf * S0;
**************************************************************************/
__Angle_Leg QrWalkScheduler::cal_kinematics(__Position P,int L,int W,int Sgn)
{
        __Angle_Leg A={0,0,0};

        double Delte = P.x - L;
        A.pitch = atan((W - P.y) / (P.z ));

        double Epsilon = L0 + P.z * cos(A.pitch) - P.y * sin(A.pitch) + W * sin(A.pitch);
        A.knee = Sgn * acos((pow(Delte,2) + pow(Epsilon,2) - pow(L1,2) - pow(L2,2)) / 2.0 / L1 / L2);

        double Phi = Delte + L2 * sin(A.knee);
        if(Phi == 0) {Phi = Phi + 0.000001;}
        A.hip = 2 * atan((Epsilon + sqrt(pow(Epsilon,2) - Phi * (L2 * sin(A.knee) - Delte))) / Phi);

        return A;
}

int QrWalkScheduler::ik(const std::vector<KDL::Frame>& footholds,
    std::vector<KDL::JntArray>& solver) {
  if (footholds.size() != ik_solvers_.size()) {
    LOG_ERROR << "The number of foothold does not match the ik solver's number";
    return -1;
  }
  readJointEncoder(temp_joint_angles_.data);

  int ret = 0;
  int ik_code = 0;
  for (int leg = 0; leg < LEG_NUM; ++leg) {
    if (temp_joint_of_leg_.data.size() != JNT_NUM_OF_EVERY_LEG[leg])
      temp_joint_of_leg_.resize(JNT_NUM_OF_EVERY_LEG[leg]);

    if (JNT_NUM_OF_EVERY_LEG[leg] != solver[leg].data.size())
      solver[leg].resize(JNT_NUM_OF_EVERY_LEG[leg]);

    int offset = 0;
    for (int i = 0; i < leg; ++i) offset += JNT_NUM_OF_EVERY_LEG[i];
    for (int jnt = 0; jnt < JNT_NUM_OF_EVERY_LEG[leg]; ++jnt) {
      temp_joint_of_leg_(jnt) = temp_joint_angles_(offset + jnt);
    }

    ik_code = ik_solvers_[leg]->CartToJnt(temp_joint_of_leg_,
                                           footholds[leg], solver[leg]);
    if (ik_code < 0) ret = ik_code;
  }

  // for debug
  std::stringstream ss;
  ss << "\n========================I K========================\n";
  for (int leg = 0; leg < LEG_NUM; ++leg) {
    for (int jnt = 0; jnt < JNT_NUM_OF_EVERY_LEG[leg]; ++jnt) {
      ss << solver[leg](jnt) << "\t";
    }
    ss << "\n";
    int offset = 0;
    for (int i = 0; i < leg; ++i) offset += JNT_NUM_OF_EVERY_LEG[i];
    for (int jnt = 0; jnt < JNT_NUM_OF_EVERY_LEG[leg]; ++jnt) {
      ss << temp_joint_angles_(offset + jnt) << "\t";
    }
    ss << "\n---------------------------------------------------\n";
  }
  ss << "===================================================";
  LOG_WARNING << ss.str();

  return ret;
}

void QrWalkScheduler::ik() {
  joint_current_positions_->lf = cal_kinematics(foots_pos_->lf, BODY_LENGTH, BODY_WIDTH,-1);
  joint_current_positions_->rf = cal_kinematics(foots_pos_->rf, BODY_LENGTH,-BODY_WIDTH,-1);
  joint_current_positions_->lb = cal_kinematics(foots_pos_->lb,-BODY_LENGTH, BODY_WIDTH, 1);
  joint_current_positions_->rb = cal_kinematics(foots_pos_->rb,-BODY_LENGTH,-BODY_WIDTH, 1);
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating CoG position and CoG adjust vector,using next foothold,while 1 means to right side
   Input: four feet(end effector) position(x,y,z),Swing leg order and next Swing leg position
   Output: CoG adjust vector
**************************************************************************/
__Position QrWalkScheduler::get_CoG_adj_vec(__Position foot_pos, unsigned int which) {
  __Position point = {0,0,0};
  switch (which) {
  case 1: // means right side
  {
    point = calcCrossPoint(foot_pos, foots_pos_->rb, foots_pos_->rf, foots_pos_->lb);
    point = calcTriangleIncentre(point, foots_pos_->rf, foots_pos_->rb);
    break;
  }
  case 2:
  {
    point = calcCrossPoint(foot_pos, foots_pos_->lb, foots_pos_->lf, foots_pos_->rb);
    point = calcTriangleIncentre(point, foots_pos_->lf, foots_pos_->lb);
    break;
  }
  default:
    break;
  }

  point.z = 0; // Default CoG = {0，0，-1 * Height}
  return point;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating line crosspoint position
   Input: four point position
   Output: crosspoint position
**************************************************************************/
__Position QrWalkScheduler::calcCrossPoint(
    const __Position& p0_0, const __Position& p0_1,
    const __Position& p1_0, const __Position& p1_1) {
  __Position cross_point = {0, 0, 0};
  Eigen::Matrix2d cof_mat;
  Eigen::Vector2d beta;
  if (p0_0.x != p0_1.x) {
    cof_mat(0, 0) = -(p0_1.y - p0_0.y) / (p0_1.x - p0_0.x);
    cof_mat(0, 1) = 1;
  } else {
    cof_mat(0, 0) = 1;
    cof_mat(0, 1) = 0;
  }
  beta(0) = cof_mat(0, 0) * p0_0.x + p0_0.y;

  if (p1_0.x != p1_1.x) {
    cof_mat(1, 0) = -(p1_1.y - p1_0.y) / (p1_1.x - p1_0.x);
    cof_mat(1, 1) = 1;
  } else {
    cof_mat(1, 0) = 1;
    cof_mat(1, 1) = 0;
  }
  beta(1) = cof_mat(1, 0) * p1_0.x + p1_0.y;

  /*std::cout << "cof_mat:" << std::endl;
    std::cout << cof_mat(0, 0) << " " << cof_mat(0, 1) << std::endl;
    std::cout << cof_mat(1, 0) << " " << cof_mat(1, 1) << std::endl;
    std::cout << "beta:" << std::endl;
    std::cout << beta(0) << " " << beta(1) << std::endl;*/
  if (0 == cof_mat.determinant()) {
    LOG_ERROR << "NO cross point";
  } else {
    Eigen::Vector2d x = cof_mat.colPivHouseholderQr().solve(beta);
    cross_point.x = x(0);
    cross_point.y = x(1);
  }

  cross_point.z = -1 * HEIGHT;
  return cross_point;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating triangle inner heart
   Input: three point position
   Output: innerheart position
**************************************************************************/
__Position QrWalkScheduler::calcTriangleIncentre(__Position A, __Position B, __Position C) {
  double a = 0, b = 0, c = 0;
  __Position heart = {0,0,0};

  a = sqrt(pow(B.x - C.x, 2) + pow(B.y - C.y, 2));
  b = sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2));
  c = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));

  heart.x = (a * A.x + b * B.x + c * C.x ) / (a + b + c);
  heart.y = (a * A.y + b * B.y + c * C.y ) / (a + b + c);
  heart.z = -1 * HEIGHT;
  return heart;
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
__Position QrWalkScheduler::get_stance_velocity(__Position Adj_vec, unsigned int Loop) {
  if(Loop > COG_COUNT) {
    LOG_ERROR << ("Time Order wrong while stance");
  }

  __Position stance_vel = {0,0,0};
  stance_vel.x = 30 * Adj_vec.x * pow(Loop, 4) / pow(COG_COUNT, 5)
      - 60 * Adj_vec.x * pow(Loop, 3) / pow(COG_COUNT, 4)
      + 30 * Adj_vec.x * pow(Loop, 2) / pow(COG_COUNT, 3);
  stance_vel.y = 30 * Adj_vec.y * pow(Loop, 4) / pow(COG_COUNT, 5)
      - 60 * Adj_vec.y * pow(Loop, 3) / pow(COG_COUNT, 4)
      + 30 * Adj_vec.y * pow(Loop, 2) / pow(COG_COUNT, 3);
  return stance_vel;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after design the exact velocity, providing accelation while move the body
   Input: CoG adjust Vector and Loop(1~)
   Output: realtime accelation
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
   X-axis:
   Acc(t):120 * XD * t^3 / Stance_Num^5 - 180 * XD * t^2 / Stance_Num^4 + 60 * XD * t / Stance_Num^3
   Y-axis:
   Acc(t):120 * YD * t^3 / Stance_Num^5 - 180 * YD * t^2 / Stance_Num^4 + 60 * YD * t / Stance_Num^3
   Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/
__Position QrWalkScheduler::get_stance_acceration(__Position Adj_vec, unsigned int Loop)
{
        if(Loop>COG_COUNT)
        {
                ROS_ERROR("Time Order wrong while stance");
        }

        __Position stance_acc = {0,0,0};
        stance_acc.x = 120 * Adj_vec.x * pow(Loop,3) / pow(COG_COUNT,5) - 180 * Adj_vec.x * pow(Loop,2) / pow(COG_COUNT,4)
                       + 60 * Adj_vec.x * Loop / pow(COG_COUNT,3);
        stance_acc.y = 120 * Adj_vec.y * pow(Loop,3) / pow(COG_COUNT,5) - 180 * Adj_vec.y * pow(Loop,2) / pow(COG_COUNT,4)
                       + 60 * Adj_vec.y * Loop / pow(COG_COUNT,3);
        return stance_acc;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: copy struct data to another one
   Input: struct data
   Output: copy data
**************************************************************************/
__Position QrWalkScheduler::struct_copy(__Position A)
{
        __Position copy_struct = {0,0,0};
        copy_struct.x = A.x;
        copy_struct.y = A.y;
        copy_struct.z = A.z;
        return copy_struct;
}
__Position QrWalkScheduler::struct_assign(double x, double y, double z)
{
        __Position copy;
        copy.x = x;
        copy.y = y;
        copy.z = z;
        return copy;
}
int QrWalkScheduler::Sgn(double a)
{
        if (a<0)
                return -1;
        else return 1;
}

void QrWalkScheduler::vec_assign(
    const Angle_Ptr& angle, std::vector<double>& vec) {
  if (vec.size() != 12) vec.resize(12);

  vec[0] = angle->lb.pitch;
  vec[1] = angle->lb.hip;
  vec[2] = angle->lb.knee;

  vec[3] = angle->lf.pitch;
  vec[4] = angle->lf.hip;
  vec[5] = angle->lf.knee;

  vec[6] = angle->rb.pitch;
  vec[7] = angle->rb.hip;
  vec[8] = angle->rb.knee;

  vec[9] = angle->rf.pitch;
  vec[10] = angle->rf.hip;
  vec[11] = angle->rf.knee;
}


} /* end for namespace qr_control */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    qr_control::QrWalkScheduler,
    controller_interface::ControllerBase)
