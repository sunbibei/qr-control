
#include "qr_control/gesture_control.h"

using namespace qr_control;

Gesture::Gesture()
{}

void Gesture::init()
{
  verbose = false;
  calibrationFlag = false; 

  Imu.roll  = 0;
  Imu.pitch = 0;
  Imu.yaw   = 0;
  
  Imu_const.roll  = 0;  
  Imu_const.pitch = 0;
  Imu_const.yaw   = 0; 
}

void Gesture::updateImuData(float roll, float pitch, float yaw)
{   
  if(!calibrationFlag)
  {
    Imu.roll  = roll /57.3248;  // X-axis
    Imu.pitch = pitch/57.3248; // Y-axis
    Imu.yaw   = yaw  /57.3248;   // Z-axis   
  }
  else
  {
    Imu.roll  = roll /57.3248  - Imu_const.roll; 
    Imu.pitch = pitch/57.3248  - Imu_const.pitch; 
    Imu.yaw   = yaw  /57.3248  - Imu_const.yaw; 
  }  
} 

void Gesture::imuCalibration()
{ 
  Imu_const.roll  = Imu.roll;
  Imu_const.pitch = Imu.pitch;
  Imu_const.yaw   = Imu.yaw;
  
  calibrationFlag = true;
  if(verbose)
  {
    std::cout<<"Gesture class: Imu Const:"<<Imu_const.roll<<", "<<Imu_const.pitch<<", "<<Imu_const.yaw<<std::endl;
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
  std::cout<<"Gesture class: Imu angle:"<<Imu.roll<<", "<<Imu.pitch<<", "<<Imu.yaw<<std::endl;
}





