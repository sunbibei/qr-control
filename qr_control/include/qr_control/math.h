
#ifndef QR_CONTROL_MATH_H_
#define QR_CONTROL_MATH_H_

#include <cmath>
#include "qr_control/ultility.h"
#include <iostream>

namespace qr_control {

  class Math {
  private:

    bool verbose;
    

  public:     

    Math();
    void init();
    _Position getInnerHeart(const _Position &A, const _Position &B, const _Position &C);
    _Position getCrossPoint(const _Position &A_start, const _Position &A_end, const _Position &B_start, const _Position &B_end);
    _Position formulaLineSection(const _Position &A, const _Position &B, float ratio);
    _Angle_Leg cal_kinematics(const _Position &P,int Sgn);
    _Position cal_formula(const _Angle_Leg &A);
    float inscribedCircleRadius(const _Position &A, const _Position &B, const _Position &C);    
    bool getJacobMatrix(const EV3& a, EM3& JacobMatrix, EM3& inverseJacobMatrix);
    EV3 jointVelToFoot(const EV3& joint_pos, const EV3& joint_vel);
    EV3 footVelToJoint(const EV3& joint_pos, const EV3& foot_vel);
    bool isJacobInvertible(const EV3& a);
    float calPos(const EV3& dist, const int& t, const int& duration);

    _Position cal_left_circul(float angle, int LegId);

  };
}
#endif

