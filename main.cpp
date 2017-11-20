/*
 * main.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */
#include <iostream>
#include "qr_control/position_joint_group_controller.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_qr");
  ros::NodeHandle nh;

  qr_control::PositionJointGroupController ctrl;
  if (!ctrl.init(nullptr, nh)) {
    std::cout << "ERROR!" << std::endl;
    return -1;
  }

  ros::Rate loop_rate(50);
  ros::Time t0 = ros::Time::now();
  ros::Time t1 = t0;

  ctrl.starting(t0);
  while (ros::ok()) {
    t1 = ros::Time::now();
    ctrl.update(t1, t1 - t0);

    t0 = t1;
    loop_rate.sleep();
  }
  return 0;
}

