
#ifndef QR_CONTROL_GESTURE_CONTROL_H_
#define QR_CONTROL_GESTURE_CONTROL_H_

#include <cmath>
#include <iostream>
#include <vector>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include "qr_control/ultility.h"

namespace qr_control {

  class Gesture:public hardware_interface::ImuSensorInterface
  {

  private:

    bool verbose;
    bool calibrationFlag;
    hardware_interface::ImuSensorHandle imu_handle_; 
    Position S = {{Body_L, Body_W, 0},{Body_L, -Body_W, 0},{-Body_L, Body_W, 0},{-Body_L, -Body_W, 0}};
    IMU Imu;
    IMU Imu_const;   
    int arr_size;
    std::vector<IMU> imu_arr;

  public:     

    Gesture();
    void init(hardware_interface::RobotHW* robot);
    void update();
    void updateImuData();
    void PostureControl();
    void imuCalibration();
    void updateShoulderPos();
    void shoulderPosControl();
    void printImuAngle();
    Position getShoulderPos();
    _Position getSingleShoulderPos(int legId);

   


    
  };
}
#endif

