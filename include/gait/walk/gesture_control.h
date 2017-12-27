
#ifndef QR_CONTROL_GESTURE_CONTROL_H_
#define QR_CONTROL_GESTURE_CONTROL_H_

#include "ultility.h"

namespace qr_control 
{

  class Gesture 
  {
  private:

    bool verbose;
    bool calibrationFlag;

    Position S = {{Body_L, Body_W, 0},{Body_L, -Body_W, 0},{-Body_L, Body_W, 0},{-Body_L, -Body_W, 0}};
    IMU Imu;
    IMU Imu_const;   

  public:     
    Gesture();
    void init();
    void updateImuData(float pitch, float yaw, float roll);
    void imuCalibration();

    IMU getImuConst();
    IMU getImuData();

    void printImuAngle();

    Position getShoulderPos();
    _Position getSingleShoulderPos(int legId);

  };
}
#endif

