/*
 * origin_test.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#include <robot/origin/origin_test.h>

namespace qr_control {

Origin* make_instance() {
  LOG_INFO << "Call QrOrigin::make_instance";

  return new OriginTest();
}

OriginTest::OriginTest() : Origin() {
  t_short_  = 0;
  t_int_    = 0;
  t_double_ = 0;
  p_thread_ = nullptr;

  t_vxd_.resize(3);
  t_mxd_.resize(4, 3);
  t_vxd_.fill(0.0);
  t_mxd_.fill(0.0);

  data_origin_.insert(std::make_pair("short", &(t_short_)));
  data_origin_.insert(std::make_pair("int",   &(t_int_)));
  data_origin_.insert(std::make_pair("double",&(t_double_)));
  data_origin_.insert(std::make_pair("vxd",   &(t_vxd_)));
  data_origin_.insert(std::make_pair("mxd",   &(t_mxd_)));

  p_thread_ = new std::thread(&OriginTest::add, this);
}

OriginTest::~OriginTest() {
  if (nullptr != p_thread_) {
    delete p_thread_;
    p_thread_ = nullptr;
  }
}

void OriginTest::add() {
  TIMER_INIT
  while (true) {
    t_short_  += 1;
    t_int_    += 10;
    t_double_ += 0.1;
    for (int i = 0; i < t_vxd_.size(); ++i)
      t_vxd_[i] += 0.01;

    for (int c = 0; c < t_mxd_.cols(); ++c)
      for (int r = 0; r < t_mxd_.rows(); ++r)
        t_mxd_(r, c) += 0.001;

    TIMER_CONTROL(500)
  }
}

} /* namespace qr_control */

/*#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::OriginTest, Label)*/
