/*
 * creep.cpp
 *
 *  Created on: Nov 21, 2017
 *      Author: bibei
 */

#include "gait/test/test_traj.h"

#include <foundation/cfg_reader.h>
#include <robot/leg/qr_leg.h>

#define SPEC_CODE 1024

#define INPUT_UNTIL_ENTER(name) \
{ \
    char c; \
    while ('\n' != (c = std::cin.get())) name.push_back(c); \
}

namespace qr_control {

TestTraj::TestTraj(const MiiString& _n)
  : GaitBase(_n), current_state_(TestTrajState::INVALID_TEST_STATE),
    state_machine_(nullptr), last_choice_(TestTrajState::STATE_INIT),
    loop_count_(0), leg_order_(LegType::UNKNOWN_LEG) {
  for (auto& l : leg_ifaces_)
    l = nullptr;
}

bool TestTraj::init() {
  if (!GaitBase::init()) return false;

  state_machine_ = new StateMachine<TestTrajState>(current_state_);
  state_machine_->registerStateCallback(
      TestTrajState::STATE_INIT,   &TestTraj::initialize, this);
  state_machine_->registerStateCallback(
      TestTrajState::STATE_READ_STATE, &TestTraj::print,  this);
  state_machine_->registerStateCallback(
      TestTrajState::STATE_COMMAND, &TestTraj::command,  this);
  state_machine_->registerStateCallback(
      TestTrajState::STATE_READ_CMD, &TestTraj::read_cmd,  this);
  state_machine_->registerStateCallback(
      TestTrajState::STATE_TRAJ_JNT, &TestTraj::traj,  this);


  current_state_ = TestTrajState::STATE_INIT;

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

  if (_DEBUG_INFO_FLAG) {
    LOG_INFO << "get interface(LegType::FL): " << leg_ifaces_[LegType::FL];
    LOG_INFO << "get interface(LegType::FR): " << leg_ifaces_[LegType::FR];
    LOG_INFO << "get interface(LegType::HL): " << leg_ifaces_[LegType::HL];
    LOG_INFO << "get interface(LegType::HR): " << leg_ifaces_[LegType::HR];
  }

  return true;
}

TestTraj::~TestTraj() {
  if (state_machine_) {
    delete state_machine_;
    state_machine_ = nullptr;
  }
}

void TestTraj::checkState() {
  /*switch(current_state_)
  {
    case TestTrajState::STATE_INIT:
    break;

    default:break;
  }

  ++loop_count_;*/
}

StateMachineBase* TestTraj::state_machine() {
  return state_machine_;
}

LegType __get_leg_type(const MiiString& _val) {
  if (0 == _val.compare("fl")) {
    return LegType::FL;
  } else if (0 ==_val.compare("fr")) {
    return LegType::FR;
  } else if (0 == _val.compare("hl")) {
    return LegType::HL;
  } else if (0 == _val.compare("hr")) {
    return LegType::HR;
  } else if (0 == _val.compare("all")) {
    return LegType::N_LEGS;
  } else {
    return LegType::UNKNOWN_LEG;
  }
}

JntType __get_jnt_type(const MiiString& _val) {
  if (0 == _val.compare("yaw")) {
    return JntType::YAW;
  } else if (0 ==_val.compare("hip")) {
    return JntType::HIP;
  } else if (0 == _val.compare("knee")) {
    return JntType::KNEE;
  } else if (0 == _val.compare("all")) {
    return JntType::N_JNTS;
  } else {
    return JntType::UNKNOWN_JNT;
  }
}

JntDataType __get_jnt_data_type(const MiiString& _val) {
  if (0 == _val.compare("pos")) {
    return JntDataType::POS;
  } else if (0 ==_val.compare("vel")) {
    return JntDataType::VEL;
  } else if (0 == _val.compare("tor")) {
    return JntDataType::TOR;
  } else if (0 == _val.compare("all")) {
    return JntDataType::N_JNT_DATA_TYPES;
  } else {
    return JntDataType::UNKNOWN_TYPE;
  }
}

bool s_is_first = true;
void TestTraj::initialize() {
  // if (s_is_first) sleep(1);
  // s_is_first = false;
  sleep(1);
  int choice = -1;
  std::string str;
  bool valid = false;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "\033[32;1m--------------------------------------------------" << std::endl;
  std::cout << "Which do you want to test?" << std::endl;
  std::cout << "1. print the state   of joints;" << std::endl;
  std::cout << "2. set   the command of joints;" << std::endl;
  std::cout << "3. read  the last command of joints(no implement);" << std::endl;
  std::cout << "4. execute the trajectory of joints(no implement);" << std::endl;
  std::cout << "5. execute the trajectory of eef(no implement);"    << std::endl;
  std::cout << "\n\nInput your choice,  or 'ctrl + c' to shutdown: \033[0m";
  INPUT_UNTIL_ENTER(str)
  choice = (str.empty()) ? (last_choice_) : (atoi(str.c_str()));
  valid = (((choice > INVALID_TEST_STATE)
      && (choice < N_TEST_TRAJ_STATE)) || (SPEC_CODE == choice));
  while (!valid) {
    std::cout << "\033[32;1mWrong! Input again, or 'ctrl + c' to shutdown: \033[0m";
    std::cin >> str;
    choice = (str.empty()) ? (last_choice_) : (atoi(str.c_str()));
    valid  = ((choice > INVALID_TEST_STATE) && (choice < N_TEST_TRAJ_STATE));
  }
  std::cout << "\033[32;1m--------------------------------------------------\033[0m" << std::endl;
  if (SPEC_CODE == choice)
    spec_code();
  else
    current_state_ = (TestTrajState)choice;

  last_choice_ = choice;
}

void TestTraj::print() {
  std::cout << "\n\n\n";
  std::cout << "\033[33;1m_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+" << std::endl;
  std::cout << "Usage: print leg type [arg1] [arg2]\033[0m" << std::endl;
  std::cout << "leg: fl, fr, hl, hr and all" << std::endl;
  std::cout << "type: pos, vel, tor and all" << std::endl;
  std::cout << "arg1: additional argument, a internal number" << std::endl;
  std::cout << "arg2: additional argument, a double number, period between each print" << std::endl;
  std::cout << "e.g." << std::endl;
  std::cout << "print fl pos 100 \033[35;1m// it will print the position of leg 'fl'.\033[0m" << std::endl;
  std::cout << "print all pos \033[35;1m// it will print the position of the all of leg.\033[0m " << std::endl;

  bool valid = false;
  bool first = true;
  do {
    if (first) std::cout << "\n\n\033[33;1mInput your choice,  or 'ctrl + c' to shutdown: \033[0m";
    else std::cout << "\n\n\033[33;1mWrong! Input again, or 'ctrl + c' to shutdown: \033[0m";
    first = false;

    std::string str;
    INPUT_UNTIL_ENTER(str)
    std::stringstream ss;
    ss << str;
    std::vector<std::string> cmds;
    while (ss >> str) {
      cmds.push_back(str);
    }

    valid = false;
    if ((cmds.size() < 3) || ((0 != cmds[0].compare("print"))
        && (0 != cmds[0].compare("p")))) continue;

    LegType     leg  = __get_leg_type(cmds[1]);
    JntDataType type = __get_jnt_data_type(cmds[2]);
    if (LegType::UNKNOWN_LEG == leg)       continue;
    if (JntDataType::UNKNOWN_TYPE == type) continue;

    valid = true;
    MiiVector<double> data;
    Eigen::VectorXd tmp;
    switch (type) {
    case JntDataType::POS:
      if (leg == LegType::N_LEGS) {
        for (auto& l : leg_ifaces_) {
          tmp = l->joint_position();
          for (int i = 0; i < tmp.size(); ++i)
            data.push_back(tmp(i));
        }
      } else {
        tmp = leg_ifaces_[leg]->joint_position();
        for (int i = 0; i < tmp.size(); ++i)
          data.push_back(tmp(i));
      }
      break;
    case JntDataType::VEL:
      if (leg == LegType::N_LEGS) {
        for (auto& l : leg_ifaces_) {
          tmp = l->joint_velocity();
          for (int i = 0; i < tmp.size(); ++i)
            data.push_back(tmp(i));
        }
      } else {
        tmp = leg_ifaces_[leg]->joint_velocity();
        for (int i = 0; i < tmp.size(); ++i)
          data.push_back(tmp(i));
      }
      break;
    case JntDataType::TOR:
      if (leg == LegType::N_LEGS) {
        for (auto& l : leg_ifaces_) {
          tmp = l->joint_torque();
          for (int i = 0; i < tmp.size(); ++i)
            data.push_back(tmp(i));
        }
      } else {
        tmp = leg_ifaces_[leg]->joint_torque();
        for (int i = 0; i < tmp.size(); ++i)
          data.push_back(tmp(i));
      }
      break;
    case JntDataType::N_JNT_DATA_TYPES:
      if (leg == LegType::N_LEGS) {
        for (auto& l : leg_ifaces_) {
          tmp = l->joint_position();
          for (int i = 0; i < tmp.size(); ++i)
            data.push_back(tmp(i));
          tmp = l->joint_velocity();
          for (int i = 0; i < tmp.size(); ++i)
            data.push_back(tmp(i));
          tmp = l->joint_torque();
          for (int i = 0; i < tmp.size(); ++i)
            data.push_back(tmp(i));
        }
      } else {
        tmp = leg_ifaces_[leg]->joint_position();
        for (int i = 0; i < tmp.size(); ++i)
          data.push_back(tmp(i));
        tmp = leg_ifaces_[leg]->joint_velocity();
        for (int i = 0; i < tmp.size(); ++i)
          data.push_back(tmp(i));
        tmp = leg_ifaces_[leg]->joint_torque();
        for (int i = 0; i < tmp.size(); ++i)
          data.push_back(tmp(i));
      }
      break;
    default: break;
    }

    int total = 1;
    if (cmds.size() >= 4) total = atoi(cmds[3].c_str());

    printf("\n\n\n\033[37;1m");
    for (int i = 0; i < total; ++i) {
      printf("%s of %s(%03d/%03d): ", cmds[2].c_str(), cmds[1].c_str(), i+1, total);
      for (int j = 0; j < data.size(); ++j)
        printf("%+01.04f ", data[j]);
      printf("\n");
      double period = 0.5;
      if (cmds.size() == 5) {
        period = atof(cmds[4].c_str());
        sleep(period);
      }
    }
    std::cout << "\033[0m";
  } while (!valid);
  std::cout << "\033[33;1m_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+\033[0m" << std::endl;
  current_state_ = TestTrajState::STATE_INIT;
}

void TestTraj::command() {
  std::cout << "\n\n\n";
  std::cout << "\033[33;1m_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+" << std::endl;
  std::cout << "Usage: set leg jnt type value [arg3]\033[0m" << std::endl;
  std::cout << "leg:   fl, fr, hl, hr and all" << std::endl;
  std::cout << "jnt:   yaw, hip, knee and all" << std::endl;
  std::cout << "type:  pos, vel, tor" << std::endl;
  std::cout << "value: a float value" << std::endl;
  std::cout << "e.g." << std::endl;
  std::cout << "set fl yaw pos 0.3 \033[35;1m// it will send the position command 0.3 to the joint 'yaw' of leg 'fl'.\033[0m" << std::endl;

  std::string str;
  bool valid = false;
  bool first = true;
  do {
    if (first) std::cout << "\n\n\033[33;1mInput your choice,  or 'ctrl + c' to shutdown: \033[0m";
    else std::cout << "\n\n\033[33;1mWrong! Input again, or 'ctrl + c' to shutdown: \033[0m";
    first = false;

    std::string str;
    INPUT_UNTIL_ENTER(str)
    std::stringstream ss;
    ss << str;
    std::vector<std::string> cmds;
    while (ss >> str) {
      cmds.push_back(str);
    }

    valid = false;
    if ((cmds.size() < 5) || ((0 != cmds[0].compare("set"))
        && (0 != cmds[0].compare("s")))) continue;

    JntType jnt      = __get_jnt_type(cmds[2]);
    LegType leg      = __get_leg_type(cmds[1]);
    JntDataType type = __get_jnt_data_type(cmds[3]);
    double val       = atof(cmds[4].c_str());
    if ((JntType::UNKNOWN_JNT    == jnt)
        || (LegType::UNKNOWN_LEG == leg)
        || (JntDataType::UNKNOWN_TYPE == type))
      continue;

    valid = true;
    switch (type) {
    case JntDataType::POS:
      if (LegType::N_LEGS == leg) {
        for (auto& iface : leg_ifaces_) {
          if (JntType::N_JNTS == jnt)
            iface->joint_command_ref().fill(val);
          else
            iface->joint_command_ref()(jnt) = val;
        }
      } else {
        if (JntType::N_JNTS == jnt)
          leg_ifaces_[leg]->joint_command_ref().fill(val);
        else
          leg_ifaces_[leg]->joint_command_ref()(jnt) = val;
      }
      std::cout << "\n\n\n\033[37;1mset " << cmds[3] << " command('" << cmds[4]
          << " into " << cmds[2] << " of " << cmds[1] << std::endl;
      break;
    case JntDataType::VEL:
    case JntDataType::TOR:
    default: std::cout << "NO IMPLEMENTED" << std::endl; break;
    }
  } while (!valid);
  std::cout << "\033[33;1m_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+\033[0m" << std::endl;
  current_state_ = TestTrajState::STATE_INIT;
}

void TestTraj::traj() {
}

void TestTraj::spec_code() {
  std::cout << "\n\n\n\033[31;1mThis is a system backdoor for you!\033[0m" << std::endl;
}

void TestTraj::read_cmd() {
  ;
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::TestTraj, Label)
