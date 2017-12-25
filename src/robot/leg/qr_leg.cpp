/*
 * qr_leg.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: bibei
 *  Implemented by Sampson on Dec 8, 2017
 */

#include "robot/leg/qr_leg.h"
#include <foundation/cfg_reader.h>
#include <foundation/auto_instanceor.h>

#include <chrono>
#include <thread>
#include <iostream>

namespace qr_control {
// params list and parser from cfg
struct __PrivateParam 
{
  size_t yaw_len;
  size_t hip_len;
  size_t knee_len;  

  __PrivateParam(const MiiString& _prefix) {
    auto cfg = MiiCfgReader::instance();
    MiiString label = Label::make_label(_prefix, "length");
    cfg->get_value(label, "yaw", yaw_len);
    cfg->get_value(label, "hip", hip_len);
    cfg->get_value(label, "knee",knee_len);   
  }
};

QrLeg::QrLeg():params_(nullptr), state_leg_(INVALID_STATE),
  td_threshold_(0)
{
  // TODO Auto-generated constructor stub
  params_ = new __PrivateParam(getLabel());
}

QrLeg::~QrLeg() 
{
  // TODO Auto-generated destructor stub
}

void QrLeg::followJntTrajectory(JntType jnt, const Trajectory1d _traj)  {
  const auto& _jnt_poss = joint_position_const_ref();
  const auto& _jnt_vels = joint_velocity_const_ref();
  // const auto& _jnt_cmds = joint_command_ref();

  // Just for debug
  for (int i = 0; i < 10; ++i) {
    std::cout << i
     << ": " << _jnt_poss(JntType::HIP)
     << ", " << _jnt_poss(JntType::HIP)
     << ", " << _jnt_poss(JntType::KNEE) << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(4));
  }



}

void QrLeg::followJntTrajectory(const Trajectory3d) {
  LOG_ERROR << "Call the 'followJntTrajectory' which has does not complemented.";
}

void QrLeg::followEefTrajectory(const Trajectory3d) 
{
  LOG_ERROR << "Call the 'followJntTrajectory' which has does not complemented.";
}

LegState QrLeg::leg_state()  {
  return state_leg_;
}

void QrLeg::printDH()
{
  LOG_INFO << "Axis No. /"<<"a(i-1) /"<<"alpha(i-1) /"<<"di /"<<"Joint Vars";
  LOG_INFO << "1    "<<"0    "<<"0    "<<"0   "<<"Theta_0";
  LOG_INFO << "2    "<<"L0   "<<"PI/2 "<<"0   "<<"Theta_1";
  LOG_INFO << "3    "<<"L1   "<<"0    "<<"0   "<<"Theta_2";
  LOG_INFO << "4    "<<"L2   "<<"0    "<<"0   "<<"0";
}

EMX QrLeg::getTransMatrixT01(const EV3& a)
{
  EMX result(4,4);
  result << cos(a(0)),-sin(a(0)),0,0,
            sin(a(0)), cos(a(0)),0,0, 
            0,         0,        1,0,
            0,         0,        0,1;
   return result;
}
EMX QrLeg::getTransMatrixT12(const EV3& a)
{
  EMX result(4,4);
  result << cos(a(1)),-sin(a(1)),0,params_->yaw_len,
            0,         0,       -1,0, 
            sin(a(1)), cos(a(1)),0,0,
            0,         0,        0,1;
  return result;
}
EMX QrLeg::getTransMatrixT23(const EV3& a)
{
  EMX result(4,4);
  result << cos(a(2)),-sin(a(2)),0,params_->hip_len,
            sin(a(2)), cos(a(2)),0,0, 
            0,         0,        1,0,
            0,         0,        0,1;
  return result;
}
EMX QrLeg::getTransMatrixT34(const EV3& a)
{
  EMX result(4,4);
  result << 1,0,0,params_->knee_len,
            0,1,0,0, 
            0,0,1,0,
            0,0,0,1;
  return result; 
}
//if ture ,the inverse is useful then
bool QrLeg::getJacobMatrix(const EV3& a, EM3& JacobMatrix, EM3& inverseJacobMatrix)
{
  bool invertible;
  float L0 = params_->yaw_len;
  float L1 = params_->hip_len;
  float L2 = params_->knee_len;  

  JacobMatrix << 0, L1*cos(a(1)) + L2*cos(a(1)+a(2)), L2*cos(a(1)+a(2)),
     L0*cos(a(0)) + L1*cos(a(0))*cos(a(1)) + L2*cos(a(0))*cos(a(1)+a(2)), 
    -L1*sin(a(0))*sin(a(1)) - L2*sin(a(0))*sin(a(1)+a(2)),
    -L2*sin(a(0))*sin(a(1)+a(2)),
     L0*sin(a(0)) + L1*sin(a(0))*cos(a(1)) + L2*sin(a(0))*cos(a(1)+a(2)), 
     L1*cos(a(0))*sin(a(1)) + L2*cos(a(0))*sin(a(1)+a(2)),
     L2*cos(a(0))*sin(a(1)+a(2));           

  JacobMatrix.computeInverseWithCheck(inverseJacobMatrix, invertible);

  return invertible;
}

EV3 QrLeg::jointVelToFoot(const EV3& joint_pos, const EV3& joint_vel)
{ 
  EM3 JacobMatrix, inverseJacobMatrix;
  bool invertible = getJacobMatrix(joint_pos, JacobMatrix, inverseJacobMatrix);
  return JacobMatrix * joint_vel;
}

EV3 QrLeg::footVelToJoint(const EV3& joint_pos, const EV3& foot_vel)
{ 
  EV3 result(-100000,-100000,-100000);
  EM3 JacobMatrix, inverseJacobMatrix;
  bool invertible = getJacobMatrix(joint_pos,JacobMatrix,inverseJacobMatrix);
  if(invertible)
  {
    result = inverseJacobMatrix * foot_vel;
  }  
  return result; 
}
EV3 QrLeg::getHipPostion(const EV3& a)
{
  EV3 result;
  EMX T = getTransMatrixT01(a);
  T = T * getTransMatrixT12(a);
  result(0) = T(0,3);
  result(1) = T(1,3);
  result(2) = T(2,3);
  return result;
}
EV3 QrLeg::getKneePostion(const EV3& a)
{
  EV3 result;
  EMX T = getTransMatrixT01(a);
  T = T * getTransMatrixT12(a);
  T = T * getTransMatrixT23(a);
  result(0) = T(0,3);
  result(1) = T(1,3);
  result(2) = T(2,3);
  return result;
}

void QrLeg::setTouchdownThreshold(const double& threshold)
{
  td_threshold_ = threshold;
}

void QrLeg::touchDownDetect()
{  
  if(foot_force() > td_threshold_)
  {    
    state_leg_ = TD_STATE;   
  }
}
void QrLeg::touchDownConditionDetect()
{  
  if(state_leg_ != TD_STATE && foot_force() > td_threshold_)
  {    
    state_leg_ = TD_STATE;   
  }
}

void QrLeg::liftDetect()
{  
  if(foot_force() < td_threshold_)
  {    
    state_leg_ = AIR_STATE;   
  }
}
void QrLeg::liftConditionDetect()
{  
  if(state_leg_ != AIR_STATE && foot_force() < td_threshold_)
  {    
    state_leg_ = AIR_STATE;   
  }
}

/*  
Description: calculating forward kinematics for 3 DOF leg using D-H methods
Formula:
   Px = L1 * S1 + L2 * S12;
   Py = L0 * S0 + L1 * S0 * C1 + L2 * S0 * C12;
   Pz = - Lo * C0 - L1 * C0 * C1 - L2 * C0 * C12;
*/
// void QrLeg::forwardKinematics(const EVX& angle, EVX& jnt_pos)
void QrLeg::forwardKinematics(Eigen::Vector3d& xyz, Eigen::Quaterniond&) {
  // TODO
  const auto& poss = joint_position_const_ref();
  xyz(0) = params_->hip_len * sin(poss(1))
                 + params_->knee_len * sin(poss(1) + poss(2));
  xyz(1) = params_->yaw_len * sin(poss(0))
                 + params_->hip_len * sin(poss(0)) * cos(poss(1))
                 + params_->knee_len * sin(poss(0)) * cos(poss(1) + poss(2));
  xyz(2) = - params_->yaw_len * cos(poss(0))
                 - params_->hip_len * cos(poss(0)) * cos(poss(1))
                 - params_->knee_len * cos(poss(0)) * cos(poss(1) + poss(2));
}
/*
Description: calculating reverse kinematics for quadruped robot(3 DOF)
Formula:
   Theta_0 = atan(- Py / Pz );
   Theta_1 = 2 * atan((Epsilon + sqrt(Epsilon^2 - Phi * (L2 * S2 - Delte))) / Phi);
   Theta_2 = sgn * acos((Delte^2 + Epsilon^2 - L1^2 - L2^2) / 2 / L1 / L2);
   Meantime, Delte = Px ; Phi = Delte + L2 * S2; Epsilon = L0 + Pz * C0 - Py * S0;
*/
// void QrLeg::inverseKinematics(const EVX& jnt_pos, EVX& angle) {
void QrLeg::inverseKinematics(const Eigen::Vector3d& xyz, Eigen::VectorXd& angle) {
  const auto& jnt_pos = joint_position_const_ref();

  int s = -1;  
  if ((LegType::HL == leg_type_) || (LegType::HR == leg_type_))
    s = 1;
  double Delte = -jnt_pos(0);
  angle(0) = atan(-jnt_pos(1) / jnt_pos(2));

  double Epsilon = params_->yaw_len + jnt_pos(2) * cos(angle(0)) - jnt_pos(1) * sin(angle(0));
  angle(2)=s*acos((pow(Delte,2)+pow(Epsilon,2)-pow(params_->hip_len,2)-pow(params_->knee_len,2))/2.0/params_->hip_len/params_->knee_len);

  double Phi = Delte + params_->knee_len * sin(angle(2));  
  Phi = (Phi==0)? 0.000001:Phi;
  
  angle(1) = 2*atan((Epsilon+sqrt(pow(Epsilon,2)-Phi*(params_->knee_len*sin(angle(2))-Delte)))/Phi);
}

void QrLeg::inverseKinematics(const Eigen::Vector3d&, const Eigen::Quaterniond&, EVX& angle) {
  LOG_ERROR << "Call the 'inverseKinematics' which has does not complemented.";
}

void QrLeg::inverseKinematics(const Eigen::Quaterniond&, EVX& angle) {
  LOG_ERROR << "Call the 'inverseKinematics' which has does not complemented.";
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::QrLeg, Label)
