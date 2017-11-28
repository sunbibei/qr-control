/*
 * comma_init.h
 *
 *  Created on: Nov 27, 2017
 *      Author: bibei
 */

#ifndef COMMA_INIT_H_
#define COMMA_INIT_H_

#include <vector>

namespace qr_control {
namespace internal {


template<typename _Obj, typename _DataType>
class CommaInitializer {
public:
  CommaInitializer(_Obj& _obj, const _DataType& s) : inst_obj_(_obj) {
    init_vec_.push_back(s);
  }

  virtual ~CommaInitializer() {
    if (init_vec_.empty()) return;

    inst_obj_ = _Obj(init_vec_);
    init_vec_.clear();
  }

public:
  CommaInitializer& operator,(const _DataType& _d) {
    init_vec_.push_back(_d);
    return *this;
  }

  CommaInitializer& operator<<(const _DataType& _d) {
    init_vec_.push_back(_d);
    return *this;
  }

protected:
  _Obj&                  inst_obj_;
  std::vector<_DataType> init_vec_;
};

} /* namespace internal */
} /* namespace qr_control */

#endif /* COMMA_INIT_H_ */
