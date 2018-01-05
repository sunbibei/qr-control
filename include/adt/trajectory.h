/*
 * trajectory.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ADT_TRAJECTORY_H_
#define INCLUDE_ADT_TRAJECTORY_H_
#include "foundation/utf.h"
#include "internal/comma_init.h"

#include <Eigen/Dense>
#include <iomanip>
#include <float.h>

namespace qr_control {

// const double INFINITY = DBL_MAX;

/*!
 * The Trajectory class is represented the Taylor expansion in two dimensional space.
 * $f(x) = a_0 + \sum_{n=1}^{n=N}{\frac{f^n(a)}{n!} \dot x^n}, so, here is
 * $f(x) = a_0 + a_1*x + a_2*x^2 + ... + a_N*x^N$
 *
 * In case three dimensional space, we use the parametric equation for a space curve.
 * Example, $ x = f1(t), y = f2(t), z=f3(t), t \in [t_min, t_max]$, The map of each
 * variable is express as a Taylor expansion. For example,
 *            | 10,  2,  3,  4   |
 *    coeff = | 0.1, 0.2 0.3 0.4 |
 *            | 1    10  20  30  |
 * The curve is like as
 *        x = 10  + 2  t + 3  t^2 + 4   t^3;
 *        y = 0.1 + 0.2t + 0.3t^2 + 0.4 t^3;
 *        z = 1   + 10 t + 20 t^2 + 30  t^3;
 *
 */
// class Trajectory { };


template<typename _DataType, int _Dim_X>
class Trajectory /*: public Trajectory*/ {
public:
  ///! Convenient alias
  typedef Eigen::Matrix<_DataType, _Dim_X, Eigen::Dynamic> CoeffMat;
  typedef Eigen::Matrix<_DataType, _Dim_X, 1>              StateVec;
  typedef Eigen::Matrix<_DataType, _Dim_X, Eigen::Dynamic> StateSeq;
  typedef struct {
    _DataType floor;
    _DataType ceiling;
  } Range;

public:
  /*!
   * @brief The default constructor which nothing to do.
   */
  Trajectory();
  /*!
   * @brief The constructor from the configure file at special tag _prefix.
   */
  Trajectory(const MiiString& _prefix);
  /*!
   * @brief The constructor.
   * @param coeff The list of coefficient.
   */
  Trajectory(const CoeffMat&);
  /*!
   * @brief The constructor.
   * @param coeff The list of coefficient.
   */
  Trajectory(const MiiVector<_DataType>&);
  /*!
   * @brief The copy constructor.
   * @param coeff The list of coefficient.
   */
  Trajectory(const Trajectory<_DataType, _Dim_X>&);
  /*!
   * @brief The assignment operators.
   * @param coeff The list of coefficient.
   */
  Trajectory<_DataType, _Dim_X>& operator=(const Trajectory<_DataType, _Dim_X>&);
  /*!
   * @brief The deconstructor
   */
  virtual ~Trajectory();

public:
  ///! This method will set the coefficients.
  virtual void reset(const CoeffMat&);
  ///! This method sample the trajectory at parameter _t
  virtual StateVec sample(_DataType _t);
  ///! This method products a sequence through discretizing the trajectory.
  virtual StateSeq sequence(_DataType _from, _DataType _to, _DataType _dt);
  ///! Differential at some point
  virtual StateVec differential(_DataType _t);
  ///! Differential trajectory object
  virtual Trajectory<_DataType, _Dim_X> differential();
  ///! TODO
  ///! Integral at some point
  // virtual StateVec integral(_DataType _t);
  ///! integral trajectory object under given sample
  virtual Trajectory<_DataType, _Dim_X> integral(const _DataType& _t0, const StateVec& _y0);

  ///! This method will clear the trajectory.
  void reset();
  ///! This method will set the range of parameters @t.
  void range(_DataType _t0, _DataType _t1);
  ///! This method return the start point
  _DataType floor(bool* exit = nullptr)   const;
  ///! This method return the end   point
  _DataType ceiling(bool* exit = nullptr) const;
protected:
  CoeffMat     coeffs_;
  Eigen::Index dim_exp_;
  Range*       range_;

public:
  template<typename _T1>
  friend std::ostream& operator<<(std::ostream& os, const Trajectory<_T1, 1>& traj);
  template<typename _T1>
  friend std::ostream& operator<<(std::ostream& os, const Trajectory<_T1, 2>& traj);
  template<typename _T1>
  friend std::ostream& operator<<(std::ostream& os, const Trajectory<_T1, 3>& traj);
  template<typename _T1, int _T2>
  friend std::ostream& operator<<(std::ostream&, const Trajectory<_T1, _T2>& traj);

  ///! Just for fun
  internal::CommaInitializer<Trajectory<_DataType, _Dim_X>, _DataType>
    operator<<(const _DataType& in);
};




///////////////////////////////////////////////////////////////////////////////
//////////////           The helper method define first
///////////////////////////////////////////////////////////////////////////////
template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1>
__get_state_vec(const _DataType& _t, const Eigen::Index& _exp);


template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1>
__get_diff_vec(const _DataType& _t, const Eigen::Index& _exp);

template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1>
__get_int_vec(const _DataType& _t, const Eigen::Index& _exp);

template<typename _DataType>
inline _DataType __clamp(const _DataType& _t, const _DataType& _low, const _DataType& _hi);

///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::Trajectory()
  : dim_exp_(0), range_(nullptr) {
  ; // Nothing to do here.
}

/*template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::Trajectory(const MiiString& _prefix)
  : dim_exp_(0)  {
  ;// TODO
}*/

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::Trajectory(const CoeffMat& _coeff)
  : coeffs_(_coeff), range_(nullptr) {
  assert(coeffs_.rows() == _Dim_X);
  dim_exp_ = coeffs_.cols();
}

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::Trajectory(const std::vector<_DataType>& _coeff)
  : range_(nullptr) {
  assert(0 == (_coeff.size()%_Dim_X));
  dim_exp_ = _coeff.size() / _Dim_X;
  coeffs_.resize(_Dim_X, dim_exp_);
  for (int r = 0; r < _Dim_X; ++r)
    for (int c = 0; c < dim_exp_; ++c)
      coeffs_(r, c) = _coeff[r*dim_exp_ + c];
}


template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::Trajectory(const Trajectory<_DataType, _Dim_X>& _o)
  : coeffs_(_o.coeffs_), dim_exp_(_o.dim_exp_), range_(nullptr) {
  if (_o.range_) {
    range_ = new Range;
    memcpy(range_, _o.range_, sizeof(Range));
  }
}

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>&
Trajectory<_DataType, _Dim_X>::operator=(const Trajectory<_DataType, _Dim_X>& _o) {
  dim_exp_ = _o.dim_exp_;
  coeffs_  = _o.coeffs_;
  delete range_;
  if (_o.range_) {
    range_ = new Range;
    memcpy(range_, _o.range_, sizeof(Range));
  } else {
    range_ = nullptr;
  }
  return *this;
}

template<typename _DataType, int _Dim_X>
void Trajectory<_DataType, _Dim_X>::reset(const CoeffMat& _new_coeff) {
  assert(_new_coeff.rows() == _Dim_X);

  coeffs_    = _new_coeff;
  dim_exp_   = coeffs_.cols();
}

template<typename _DataType, int _Dim_X>
void Trajectory<_DataType, _Dim_X>::reset() {
  coeffs_.fill(0.0);
  dim_exp_ = 0;
  delete range_;
  range_ = nullptr;
}

template<typename _DataType, int _Dim_X>
void Trajectory<_DataType, _Dim_X>::range(_DataType _t0, _DataType _t1) {
  if (!range_) range_ = new Range;

  range_->floor   = _t0;
  range_->ceiling = _t1;
}

template<typename _DataType, int _Dim_X>
_DataType Trajectory<_DataType, _Dim_X>::floor(bool* exit) const {
  if (exit) *exit = (nullptr != range_);

  return (range_) ? range_->floor : std::numeric_limits<_DataType>::min();
}

template<typename _DataType, int _Dim_X>
_DataType Trajectory<_DataType, _Dim_X>::ceiling(bool* exit) const {
  if (exit) *exit = (nullptr != range_);

  return (range_) ? range_->ceiling : std::numeric_limits<_DataType>::max();
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateVec
Trajectory<_DataType, _Dim_X>::sample(_DataType _t) {
  assert(0 != coeffs_.cols());
  if (range_) _t = __clamp(_t, range_->floor, range_->ceiling);

  return coeffs_ * __get_state_vec<_DataType>(_t, dim_exp_);
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateSeq
Trajectory<_DataType, _Dim_X>::sequence(
    _DataType _from, _DataType _to, _DataType _dt) {
  assert((_to > _from) && (0 != _dt));
  if (range_) {
    if (_from < range_->floor)   _from = range_->floor;
    if (_to   > range_->ceiling) _to   = range_->ceiling;
  }
  typename Trajectory<_DataType, _Dim_X>::StateSeq ret;
  ret.resize(_Dim_X, (_to - _from) / _dt + 1);

  Eigen::Index off = 0;
  for (_DataType _t = _from; _t < _to; _t += _dt)
    ret.col(off++) = sample(_t);

  return ret;
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateVec
Trajectory<_DataType, _Dim_X>::differential(_DataType _t) {

  return coeffs_ * __get_diff_vec(_t, dim_exp_);
}

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>
Trajectory<_DataType, _Dim_X>::differential() {
  Trajectory<_DataType, _Dim_X> ret(*this);
  --ret.dim_exp_;
  ret.coeffs_.resize(Eigen::NoChange, ret.dim_exp_);


  for (auto col = 1; col < coeffs_.cols(); ++col)
    ret.coeffs_.col(col-1) = col*coeffs_.col(col);

  return ret;
}

//template<typename _DataType, int _Dim_X>
//typename Trajectory<_DataType, _Dim_X>::StateVec
//Trajectory<_DataType, _Dim_X>::integral(_DataType _t) {
//  return coeffs_ * __get_int_vec(_t, dim_exp_);
//}

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>
Trajectory<_DataType, _Dim_X>::integral(const _DataType& _t0, const StateVec& _y0) {
  Trajectory<_DataType, _Dim_X> ret(*this);
  ++ret.dim_exp_;
  ret.coeffs_.resize(Eigen::NoChange, ret.dim_exp_);

  ret.coeffs_.col(0).fill(0);
  for (auto col = 1; col <= coeffs_.cols(); ++col)
    ret.coeffs_.col(col) = coeffs_.col(col - 1) / col;

  ret.coeffs_.col(0) = _y0 - ret.sample(_t0);
  return ret;
}

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::~Trajectory() {
  ;// Nothing to do here.
}

template<typename _T1, int _T2>
std::ostream& operator<<(std::ostream& os, const Trajectory<_T1, _T2>& traj) {
  os.setf(std::ios::showpos | std::ios::fixed/* | std::ios::internal*/);
  for (int i = 0; i < _T2; ++i) {
    os << "x" << std::to_string(i) << " = ";
    const auto& _coeff = traj.coeffs_.row(i);
    for (int j = 0; j < _coeff.cols(); ++j) {
      os << std::setw(10) << std::setprecision(2) << _coeff(j);
      os << " t^" << std::to_string(j);
    }
    if (traj.range_) os << ",\tt \\in (" << traj.range_->floor
        << ", " << traj.range_->ceiling << ")";
    os << "\n";
  }
  return os;
}

template<typename _T1>
std::ostream& operator<<(std::ostream& os, const Trajectory<_T1, 1>& traj) {
  os.setf(std::ios::showpos | std::ios::fixed/* | std::ios::internal*/);

  os << "x = ";
  const auto& _coeff = traj.coeffs_.row(0);
  for (int j = 0; j < _coeff.cols(); ++j) {
    os << std::setw(10) << std::setprecision(2) << _coeff(j);
    os << " t^" << std::to_string(j);
  }
  if (traj.range_) os << ",\tt \\in (" << traj.range_->floor
      << ", " << traj.range_->ceiling << ")";
  os << "\n";

  return os;
}

template<typename _T1>
std::ostream& operator<<(std::ostream& os, const Trajectory<_T1, 2>& traj) {
  os.setf(std::ios::showpos | std::ios::fixed/* | std::ios::internal*/);
  for (int i = 0; i < 2; ++i) {
    os << (const char* []) {"x = ", "y = "}[i];
    const auto& _coeff = traj.coeffs_.row(i);
    for (int j = 0; j < _coeff.cols(); ++j) {
      os << std::setw(10) << std::setprecision(2) << _coeff(j);
      os << " t^" << std::to_string(j);
    }
    if (traj.range_) os << ",\tt \\in (" << traj.range_->floor
        << ", " << traj.range_->ceiling << ")";
    os << "\n";
  }
  // os << "The coefficients of the trajectory is :\n" << traj.coeffs_ << std::endl;
  return os;
}

template<typename _T1>
std::ostream& operator<<(std::ostream& os, const Trajectory<_T1, 3>& traj) {
  os.setf(std::ios::showpos | std::ios::fixed/* | std::ios::internal*/);
  for (int i = 0; i < 3; ++i) {
    os << (const char* []) {"x = ", "y = ", "z = "}[i];
    const auto& _coeff = traj.coeffs_.row(i);
    for (int j = 0; j < _coeff.cols(); ++j) {
      os << std::setw(10) << std::setprecision(2) << _coeff(j);
      os << " t^" << std::to_string(j);
    }
    if (traj.range_) os << ",\tt \\in (" << traj.range_->floor
        << ", " << traj.range_->ceiling << ")";
    os << "\n";
  }
  // os << "The coefficients of the trajectory is :\n" << traj.coeffs_ << std::endl;
  return os;
}

template<typename _DataType, int _Dim_X>
internal::CommaInitializer<Trajectory<_DataType, _Dim_X>, _DataType>
Trajectory<_DataType, _Dim_X>::operator<<(const _DataType& s) {
  return internal::CommaInitializer<Trajectory<_DataType, _Dim_X>, _DataType>(*this, s);
}



///////////////////////////////////////////////////////////////////////////////
////////        The implementation of template helper methods         /////////
///////////////////////////////////////////////////////////////////////////////
template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1> __get_state_vec(const _DataType& _t, const Eigen::Index& _exp) {
  Eigen::Matrix<_DataType, Eigen::Dynamic, 1> _vec;
  _vec.resize(_exp, 1);
  Eigen::Index idx = 0;
  _vec(idx++) = 1;
  for (; idx < _vec.rows(); ++idx) {
    _vec(idx) = _vec(idx-1)*_t;
  }
  return _vec;
}

template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1>
__get_diff_vec(const _DataType& _t, const Eigen::Index& _exp) {
  Eigen::Matrix<_DataType, Eigen::Dynamic, 1> _vec;
  _vec.fill(0);
  _vec.resize(_exp, 1);
  if (_exp >= 2) {
    _vec(0) = 0;
    _vec(1) = 1;
    for (Eigen::Index idx = 2; idx < _vec.rows(); ++idx)
      _vec(idx) = _vec(idx-1)*_t;
    for (Eigen::Index idx = 2; idx < _vec.rows(); ++idx)
      _vec(idx) *= idx;
  }
  return _vec;
}

template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1>
__get_int_vec(const _DataType& _t, const Eigen::Index& _exp) {
  Eigen::Matrix<_DataType, Eigen::Dynamic, 1> _vec;
  _vec.resize(_exp, 1);
  _vec(0) = _t;
  for (Eigen::Index idx = 1; idx < _vec.rows(); ++idx)
    _vec(idx) = _vec(idx-1)*_t;

  for (Eigen::Index idx = 1; idx < _vec.rows(); ++idx)
    _vec(idx) *= (1/(idx+1));
  return _vec;
}

template<typename _DataType>
inline _DataType __clamp(const _DataType& _t, const _DataType& _low, const _DataType& _hi) {
  return ( (_low < _hi) ? ( (_t < _low) ? _low : ( (_t > _hi ) ? (_hi ) : _t ) )
                        : ( (_t < _hi)  ? _hi  : ( (_t > _low) ? (_low) : _t ) ) );
}


#define TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
using Trajectory##SizeSuffix##TypeSuffix = Trajectory<Type, Size>;

#define TRAJ_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 1, 1) \
TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4) \

TRAJ_MAKE_TYPEDEFS_ALL_SIZES(int,    i)
TRAJ_MAKE_TYPEDEFS_ALL_SIZES(char,   c)
TRAJ_MAKE_TYPEDEFS_ALL_SIZES(short,  s)
TRAJ_MAKE_TYPEDEFS_ALL_SIZES(float,  f)
TRAJ_MAKE_TYPEDEFS_ALL_SIZES(double, d)

#undef TRAJ_MAKE_TYPEDEFS_ALL_SIZES
#undef TRAJ_MAKE_TYPEDEFS

} /* namespace qr_control */

#endif /* INCLUDE_ADT_TRAJECTORY_H_ */
