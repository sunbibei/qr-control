/*
 * robot_body.h
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_ROBOT_BODY_H_
#define INCLUDE_ROBOT_ROBOT_BODY_H_

#include <foundation/label.h>
#include <foundation/utf.h>
#include <Eigen/Dense>

#define G 9.80665

namespace qr_control {

class RobotBody: public Label {
public:
  RobotBody() : Label("robot-body") { };
  virtual ~RobotBody() { };

public:
  
  void cogCal(EV3& translation);
 
  void rotation(Eigen::Quaterniond& quaternion);
  
  void velocity(EV3& v);

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

#endif /* INCLUDE_ROBOT_ROBOT_BODY_H_ */
