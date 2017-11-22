/*
 * origin.h
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_ORIGIN_H_
#define INCLUDE_ROBOT_ORIGIN_H_

#include <system/foundation/label.h>
#include <boost/variant.hpp>
#include <Eigen/Dense>

namespace qr_control {

typedef boost::variant<const short*, const int*, const double*,
    const Eigen::VectorXi*, const Eigen::MatrixXi*,
    const Eigen::VectorXd*, const Eigen::MatrixXd*> DataTypeFrom;

typedef boost::variant<short*, int*, double*,
    Eigen::VectorXi*, Eigen::MatrixXi*,
    Eigen::VectorXd*, Eigen::MatrixXd*> DataTypeTo;

class Origin : public Label {
  SINGLETON_DECLARE(Origin)

public:
  /*template<typename _DataType>
  _DataType get(const MiiString&);

private:*/
  ///! The boost static assert fail! so we need split into two methods.
  template<typename _DataType>
  _DataType resource(const MiiString&);

  template<typename _DataType>
  _DataType command(const MiiString&);

protected:
  MiiMap<MiiString, DataTypeFrom> data_origin_;
  MiiMap<MiiString, DataTypeTo>   cmd_origin_;
};

/*!
 * @brief This method must be implemented in the sub-class.
 *        This is a factory method.
 */
Origin* make_instance();

///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
/*template <typename _DataType>
_DataType Origin::get(const MiiString& _res_name) {
  if (data_origin_.end() != data_origin_.find(_res_name)) {
    return getFrom<_DataType>(_res_name);
  } else if (cmd_origin_.end() != cmd_origin_.find(_res_name)) {
    return getTo<_DataType>(_res_name);
  } else {
    LOG_ERROR << "No such named resource: " << _res_name;
    assert(false);
  }
}*/

template <typename _DataType>
_DataType Origin::resource(const MiiString& _res_name) {
  if (data_origin_.end() == data_origin_.find(_res_name)) {
    return _DataType(nullptr);
  }
  auto var_data = data_origin_[_res_name];
  assert(var_data.type() == typeid(_DataType));

  return boost::get<_DataType>(var_data);
}

template <typename _DataType>
_DataType Origin::command(const MiiString& _res_name) {
  if (cmd_origin_.end() != cmd_origin_.find(_res_name)) {
    return _DataType(nullptr);
  }

  auto var_cmd = cmd_origin_[_res_name];
  assert(var_cmd.type() == typeid(_DataType));

  return boost::get<_DataType>(var_cmd);
}

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_ORIGIN_H_ */
