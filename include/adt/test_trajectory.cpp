/*
 * test_trajectory.cpp
 *
 *  Created on: Dec 8, 2017
 *      Author: bibei
 */

#ifdef __TEST_TRAJ_
#include "trajectory.h"

#include <iostream>

using namespace qr_control;

int main() {
  Trajectory3d trj2d;
  Trajectory3d::CoeffMat coeff;
  Eigen::Vector4d c(0.1, 0.2, 0.3, 0.4);
  coeff.resize(3, 4);
  coeff << 0.1, 0.2, 0.3,
      0.4, 0.5, 0.6,
      0.7, 0.8, 0.9,
      1.0, 1.1, 1.2;
  /*coeff.row(0) = c.transpose();
  coeff.row(1) = Eigen::Vector4d(1, 2, 3, 4);
  coeff.row(2) = Eigen::Vector4d(10, 20, 30, 40);*/
  trj2d.reset(coeff);

  std::cout << trj2d;
  std::cout << "The differential at 0.1 is\n" << trj2d.differential(0.1) << std::endl;
  auto diff = trj2d.differential();
  std::cout << diff;
  std::cout << "The differential at 0.1 is\n" << diff.sample(0.1) << std::endl;

  trj2d << 11, 22, 33,
      44, 55, 66;
  std::cout << trj2d << std::endl;
  std::cout << "The differential at 0.1 is\n" << trj2d.differential(0.1) << std::endl;
  auto diff1 = trj2d.differential();
  std::cout << diff1;
  std::cout << "The differential at 0.1 is\n" << diff1.sample(0.1) << std::endl;

  std::cout << "sample: \n" << trj2d.sample(0.1) << std::endl;
  std::cout << "sample: \n" << trj2d.sample(1) << std::endl;
  std::cout << "sample: \n" << trj2d.sample(10) << std::endl;

  std::cout << "sequence: \n" << trj2d.sequence(0.1, 1, 0.1) << std::endl;

  std::cout << "END" << std::endl;
  return 0;

}
#endif
