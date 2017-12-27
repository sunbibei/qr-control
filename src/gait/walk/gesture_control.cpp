
#include "gait/walk/gesture_control.h"

using namespace qr_control;

Gesture::Gesture()
{}

void Gesture::init()
{
  verbose = false;
  calibrationFlag = false; 

  Imu.pitch = 0;
  Imu.yaw = 0;
  Imu.roll = 0;

  Imu_const.pitch = 0;
  Imu_const.yaw = 0;
  Imu_const.roll = 0;  
}

void Gesture::updateImuData(float pitch, float yaw, float roll)
{   
  if(!calibrationFlag)
  {
    Imu.pitch = pitch/57.3248; // X-axis
    Imu.yaw   = yaw/57.3248; // Y-axis
    Imu.roll  = roll/57.3248; // Z-axis
  }
  else
  {
    Imu.pitch = pitch/57.3248 - Imu_const.pitch; // X-axis
    Imu.yaw   = yaw/57.3248 - Imu_const.yaw; // Y-axis
    Imu.roll  = roll/57.3248 - Imu_const.roll; // Z-axis
  }  

  if(verbose)
  {
    std::cout<<"Gesture class: IMU :"<<round(Imu.pitch*1000)/1000<<", "<<round(Imu.yaw *1000)/1000<<", "<<round(Imu.roll*1000)/1000<<std::endl;
  }
} 

void Gesture::imuCalibration()
{ 
  Imu_const.pitch = Imu.pitch;
  Imu_const.yaw = Imu.yaw;
  Imu_const.roll = Imu.roll;
  calibrationFlag = true;
  if(verbose)
  {
    std::cout<<"Gesture class: Imu Const:"<<Imu_const.pitch<<", "<<Imu_const.yaw<<", "<<Imu_const.roll<<std::endl;
  }
}

IMU Gesture::getImuConst()
{
  return Imu_const; 
}

IMU Gesture::getImuData()
{
  return Imu; 
}

Position Gesture::getShoulderPos()
{
  return S;
}

_Position Gesture::getSingleShoulderPos(int legId)
{
  switch(legId)
  {
    case LF:
    return S.lf;  
    case RF:
    return S.rf;
    case LB:
    return S.lb;
    case RB:
    return S.rb;  
  } 
}

void Gesture::printImuAngle()
{
  std::cout<<"Gesture class: Imu angle:"<<Imu.pitch<<", "<<Imu.yaw<<", "<<Imu.roll<<std::endl;
}





