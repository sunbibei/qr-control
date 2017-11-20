
#include "qr_control/gesture_control.h"

using namespace qr_control;

Gesture::Gesture()
{}

void Gesture::init(hardware_interface::RobotHW* robot)
{
  verbose = false;
  calibrationFlag = false;
  std::string str = "";
  if (ros::param::get("imu", str)) 
  {
    ImuSensorInterface* imu = robot->get<ImuSensorInterface>();
    std::cout << "Get IMU Name: " << str << std::endl;
    imu_handle_ = imu->getHandle(str);
  }

  Imu.pitch = 0;
  Imu.yaw = 0;
  Imu.roll = 0;  
  Imu_const.pitch = 0;
  Imu_const.yaw = 0;
  Imu_const.roll = 0;

  arr_size = 100;
  for(int i=0;i<arr_size;i++)
  {
    imu_arr.push_back(Imu);
  }
  
}

void Gesture::updateImuData()
{   
  auto d = this->imu_handle_.getOrientation();
  if(!calibrationFlag)
  {
    Imu.pitch = d[0]/57.3248; // X-axis
    Imu.yaw   = d[1]/57.3248; // Y-axis
    Imu.roll  =-d[2]/57.3248; // Z-axis
  }
  else
  {
    Imu.pitch = d[0]/57.3248 - Imu_const.pitch; // X-axis
    Imu.yaw   = d[1]/57.3248 - Imu_const.yaw; // Y-axis
    Imu.roll  =-d[2]/57.3248 - Imu_const.roll; // Z-axis
  }
  for(int i=1;i<arr_size;i++)
  {
    imu_arr[i-1] = imu_arr[i];
  }
  imu_arr[arr_size-1] = Imu;

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

void Gesture::PostureControl()
{

}

void Gesture::updateShoulderPos()//float pitch,float yaw,float roll
{
  S.rf.x = S.lf.x = Body_L*cos(Imu.yaw);
  S.rb.x = S.lb.x = -S.lf.x;
  S.lb.y = S.lf.y = Body_W*cos(Imu.pitch);  
  S.rb.y = S.rf.y = - S.lf.y; 
  S.rf.z = S.lf.z = Body_L*sin(-Imu.yaw)*cos(Imu.pitch) + Body_W*sin(Imu.pitch);
  S.rb.z = S.lb.z = -S.lf.z;
 
  if(verbose)
  {
    std::cout<<"LF:"<<S.lf.x<<","<<S.lf.y<<", "<<S.lf.z<<std::endl;
    std::cout<<"RF:"<<S.rf.x<<","<<S.rf.y<<", "<<S.rf.z<<std::endl;
    std::cout<<"LB:"<<S.lb.x<<","<<S.lb.y<<", "<<S.lb.z<<std::endl;
    std::cout<<"RB:"<<S.rb.x<<","<<S.rb.y<<", "<<S.rb.z<<std::endl;
  }
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





