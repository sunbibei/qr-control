/*
 * robot_body.h
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_BODY_ROBOT_BODY_H_
#define INCLUDE_ROBOT_BODY_ROBOT_BODY_H_

#include "math_body.h"

namespace qr_control {

class RobotBody: public MathBody {
public:
  RobotBody();
  // virtual bool init() override;
  virtual ~RobotBody();

///! inherit from MathBody
public:
  ///! The translation of robot against the world frame
  void translation(EV3&) /*= 0*/;
  ///! The rotation of robot against the world frame
  void rotation(Eigen::Quaterniond&) /*= 0*/;
  ///! The velocity of robot against the world frame
  void velocity(EV3& v)  /*= 0*/;
  ///! The centre of gravity of robot
  void cog(EV3&)         /*= 0*/;

public:
  void calZmpPos();
  void setExecuteDuration(const double& duration);
  void setCogThreshold(const double& threshold);

  EV3 calCogPos(const EV3& dist, const int& t);
  EV3 calCogVel(const EV3& dist, const int& t);

private:
  EV3 calInnerHeart(const EV3& A, const EV3& B, const EV3& C);

  EV3 calLineSection(const EV3& A, const EV3& B, float ratio);  

  EV3 calCrossPoint(const EV3& P1, const EV3& P2, const EV3& P3, const EV3& P4);

  bool calInnerTriangle(const EV3& A, const EV3& B, const EV3& C, EM3& Triangle);

  float calInscribedCircleRadius(const EV3& A, const EV3& B, const EV3& C);

private:
  EV3 body_cog_, body_vel_, body_acc, body_zmp_;
  float cog_threshold_, duration_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_BODY_ROBOT_BODY_H_ */
