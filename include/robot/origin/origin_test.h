/*
 * origin_test.h
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_ORIGIN_ORIGIN_TEST_H_
#define INCLUDE_ROBOT_ORIGIN_ORIGIN_TEST_H_

#include <robot/origin/origin.h>
#include <thread>

namespace qr_control {

class OriginTest: public Origin {
  friend Origin* make_instance();
protected:
  OriginTest(/*const MiiString& _prefix*/);
  virtual ~OriginTest();

private:
  void add();
  short  t_short_;
  int    t_int_;
  double t_double_;
  Eigen::VectorXd t_vxd_;
  Eigen::MatrixXd t_mxd_;
  std::thread*    p_thread_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_ORIGIN_ORIGIN_TEST_H_ */
