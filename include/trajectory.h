/*
 * trajectory.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_TRAJECTORY_H_
#define INCLUDE_TRAJECTORY_H_
#include <system/foundation/utf.h>

namespace qr_control {

/*!
 * The Trajectory class is represented the Taylor expansion.
 * $f(x) = a_0 + \sum_{n=1}^{n=N}{\frac{f^n(a)}{n!} \dot x^n}
 * so, here is $f(x) = a_0 + a_1*x + a_2*x^2 + ... + a_N*x^N$
 */
template<typename _DataType>
class Trajectory {
public:
  Trajectory();
  /*!
   * @brief The constructor.
   * @param coeff The list of coefficient.
   */
  Trajectory(const MiiVector<double>&);
  virtual ~Trajectory();

public:
  ///! This method will set the coefficients.
  void reset(const MiiVector<double>&);
  ///! This method sample the trajectory at _x
  _DataType sample(const _DataType& _x);
  ///! This method products a sequence through discretizing the trajectory.
  MiiVector<_DataType> sequence(_DataType _dx, _DataType _from, _DataType _to);

protected:
  MiiVector<_DataType> coefficients_;
  _DataType x_floor_;
  _DataType x_ceiling_;
};


///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template <typename _DataType>
_DataType Trajectory<_DataType>::sample(const _DataType& _x) {
  _DataType ret = ;
}

} /* namespace qr_control */

#endif /* INCLUDE_TRAJECTORY_H_ */
