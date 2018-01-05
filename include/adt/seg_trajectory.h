/*
 * seg_trajectory.h
 *
 *  Created on: Jan 5, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ADT_SEG_TRAJECTORY_H_
#define INCLUDE_ADT_SEG_TRAJECTORY_H_

#include "trajectory.h"

namespace qr_control {

/*!
 * @brief The implement of Segmented trajectory.
 */
template<typename _DataType, int _Dim_X>
class SegTrajectory/*: protected Trajectory<_DataType, _Dim_X>*/ {
public:
  ///! Convenient alias
  typedef Eigen::Matrix<_DataType, _Dim_X, Eigen::Dynamic> CoeffMat;
  typedef Eigen::Matrix<_DataType, _Dim_X, 1>              StateVec;
  typedef Eigen::Matrix<_DataType, _Dim_X, Eigen::Dynamic> StateSeq;

public:
  /*!
   * @brief The default constructor.
   */
  SegTrajectory();

  virtual ~SegTrajectory();

public:
  ///! This method sample the trajectory at parameter _t
  virtual StateVec sample(_DataType _t);
  ///! This method products a sequence through discretizing the trajectory.
  virtual StateSeq sequence(_DataType _from, _DataType _to, _DataType _delta);
  ///! Differential at some point
  // virtual StateVec differential(_DataType _t) override;
  ///! Differential trajectory object
  // virtual Trajectory<_DataType, _Dim_X> differential() override;
  ///! integral trajectory object under given sample
  // virtual Trajectory<_DataType, _Dim_X> integral(const _DataType& _t0, const StateVec& _y0) override;

  ///! This method will set the coefficients.
  void add(const CoeffMat&, _DataType _from, _DataType _to);
  void add(const Trajectory<_DataType, _Dim_X>&, _DataType _from, _DataType _to);
  void add(const Trajectory<_DataType, _Dim_X>&);
  ///! This method remove a segmented trajectory
  // void remove(Trajectory<_DataType, _Dim_X>*);
  ///! This method will clear the trajectory.
  void reset();
  ///! This method return the start point
  _DataType floor(bool* exit = nullptr);
  ///! This method return the end   point
  _DataType ceiling(bool* exit = nullptr);

public:
  template<typename _T1, int _T2>
  friend std::ostream& operator<<(std::ostream&, const SegTrajectory<_T1, _T2>& traj);

protected:
  MiiVector<Trajectory<_DataType, _Dim_X>*>                 segs_;
  MiiVector<typename Trajectory<_DataType, _Dim_X>::Range*> ranges_;
  typename Trajectory<_DataType, _Dim_X>::Range*            coj_range_;
};



///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
const size_t DEFAULT_SEGS = 8;
template<typename _DataType, int _Dim_X>
SegTrajectory<_DataType, _Dim_X>::SegTrajectory() {
  segs_.reserve(DEFAULT_SEGS);
  // for (auto& s : segs_)   s = nullptr;

  ranges_.reserve(DEFAULT_SEGS);
  // for (auto& r : ranges_) r = nullptr;
  coj_range_ = nullptr;
}

template<typename _DataType, int _Dim_X>
SegTrajectory<_DataType, _Dim_X>::~SegTrajectory() {
  reset();
}

template<typename _DataType, int _Dim_X>
void SegTrajectory<_DataType, _Dim_X>::add(
    const CoeffMat& _new_coeff, _DataType _from, _DataType _to) {
  auto tmp = new Trajectory<_DataType, _Dim_X>(_new_coeff);
  tmp->range(_from, _to);
  segs_.push_back(tmp);

  auto tmp_range = new typename Trajectory<_DataType, _Dim_X>::Range;
  tmp_range->floor   = _from;
  tmp_range->ceiling = _to;
  ranges_.push_back(tmp_range);

  if (!coj_range_) {
    coj_range_ = new typename Trajectory<_DataType, _Dim_X>::Range;
    coj_range_->floor   = _from;
    coj_range_->ceiling = _to;
  } else {
    coj_range_->floor   = std::min(coj_range_->floor,   _from);
    coj_range_->ceiling = std::max(coj_range_->ceiling, _to);
  }
}

template<typename _DataType, int _Dim_X>
void SegTrajectory<_DataType, _Dim_X>::add(
    const Trajectory<_DataType, _Dim_X>& _new_traj) {
  auto tmp = new Trajectory<_DataType, _Dim_X>(_new_traj);
  segs_.push_back(tmp);

  auto tmp_range = new typename Trajectory<_DataType, _Dim_X>::Range;
  tmp_range->floor   = _new_traj.floor();
  tmp_range->ceiling = _new_traj.ceiling();
  ranges_.push_back(tmp_range);

  if (!coj_range_) {
    coj_range_ = new typename Trajectory<_DataType, _Dim_X>::Range;
    coj_range_->floor   = _new_traj.floor();
    coj_range_->ceiling = _new_traj.ceiling();
  } else {
    coj_range_->floor   = std::min(coj_range_->floor,   _new_traj.floor());
    coj_range_->ceiling = std::max(coj_range_->ceiling, _new_traj.ceiling());
  }
}

template<typename _DataType, int _Dim_X>
void SegTrajectory<_DataType, _Dim_X>::add(
    const Trajectory<_DataType, _Dim_X>& _new_traj, _DataType _from, _DataType _to) {
  auto tmp = new Trajectory<_DataType, _Dim_X>(_new_traj);
  tmp->range(_from, _to);
  segs_.push_back(tmp);

  auto tmp_range = new typename Trajectory<_DataType, _Dim_X>::Range;
  tmp_range->floor   = _from;
  tmp_range->ceiling = _to;
  ranges_.push_back(tmp_range);

  if (!coj_range_) {
    coj_range_ = new typename Trajectory<_DataType, _Dim_X>::Range;
    coj_range_->floor   = _from;
    coj_range_->ceiling = _to;
  } else {
    coj_range_->floor   = std::min(coj_range_->floor,   _from);
    coj_range_->ceiling = std::max(coj_range_->ceiling, _to);
  }
}

//template<typename _DataType, int _Dim_X>
//void SegTrajectory<_DataType, _Dim_X>::remove(Trajectory<_DataType, _Dim_X>* addr) {
//  for (size_t i = 0; i < segs_.size(); ++i)   {
//    if (segs_[i] == addr) {
//      delete segs_[i];
//      segs_[i] = nullptr;
//
//      delete ranges_[i];
//      ranges_[i] = nullptr;
//    }
//  }
//}

template<typename _DataType, int _Dim_X>
void SegTrajectory<_DataType, _Dim_X>::reset() {
  for (auto& s : segs_)   {
    delete s;
    s = nullptr;
  }
  segs_.clear();

  for (auto& r : ranges_)   {
    delete r;
    r = nullptr;
  }
  ranges_.clear();

  delete coj_range_;
  coj_range_ = nullptr;
}

template<typename _DataType, int _Dim_X>
_DataType SegTrajectory<_DataType, _Dim_X>::floor(bool* exit) {
  if (exit) *exit = (nullptr != coj_range_);

  return (coj_range_) ? coj_range_->floor : std::numeric_limits<_DataType>::min();
}

template<typename _DataType, int _Dim_X>
_DataType SegTrajectory<_DataType, _Dim_X>::ceiling(bool* exit) {
  if (exit) *exit = (nullptr != coj_range_);

  return (coj_range_) ? coj_range_->ceiling : std::numeric_limits<_DataType>::max();
}

template<typename _DataType, int _Dim_X>
typename SegTrajectory<_DataType, _Dim_X>::StateVec
SegTrajectory<_DataType, _Dim_X>::sample(_DataType _t) {
  _t = __clamp(_t, coj_range_->floor, coj_range_->ceiling);

  for (size_t i = 0; i < ranges_.size(); ++i)   {
    if ((_t >= ranges_[i]->floor) && (_t <= ranges_[i]->ceiling)) {
      return segs_[i]->sample(_t);
    }
  }

  ///! Don't should be here.
  assert(false && ("range error"));
  return segs_[0]->sample(_t);
}

template<typename _DataType, int _Dim_X>
typename SegTrajectory<_DataType, _Dim_X>::StateSeq
SegTrajectory<_DataType, _Dim_X>::sequence(
    _DataType _from, _DataType _to, _DataType _dt) {
  assert((_to > _from) && (0 != _dt));
  typename SegTrajectory<_DataType, _Dim_X>::StateSeq ret;
  ret.resize(_Dim_X, (_to - _from) / _dt + 1);

  Eigen::Index off = 0;
  for (_DataType _t = _from; _t < _to; _t += _dt)
    ret.col(off++) = sample(_t);

  return ret;
}

template<typename _T1, int _T2>
std::ostream& operator<<(std::ostream& os, const SegTrajectory<_T1, _T2>& traj) {
  for (const auto& t : traj.segs_) {
    os << *t;
  }
  return os;
}

#define SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
using SegTrajectory##SizeSuffix##TypeSuffix = SegTrajectory<Type, Size>;

#define SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
    SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 1, 1) \
    SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
    SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
    SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4) \

SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(int,    i)
SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(char,   c)
SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(short,  s)
SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(float,  f)
SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(double, d)

#undef SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES
#undef SEG_TRAJ_MAKE_TYPEDEFS

} /* namespace qr_control */

#endif /* INCLUDE_ADT_SEG_TRAJECTORY_H_ */
