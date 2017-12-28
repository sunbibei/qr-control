/*
 * data_body.cpp
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#include <robot/body/data_body.h>
#include <foundation/cfg_reader.h>
#include <repository/registry.h>

namespace qr_control {

struct _ImuSensor {
  const Eigen::VectorXd* ang_vel_;
  const Eigen::MatrixXd* ang_vel_cov_;
  const Eigen::VectorXd* lin_acc_;
  const Eigen::MatrixXd* lin_acc_cov_;
  const Eigen::VectorXd* quat_;
  const Eigen::MatrixXd* quat_cov_;

  _ImuSensor(const MiiString& tag) {
    auto cfg = MiiCfgReader::instance();
    MiiVector<MiiString> strs;
    cfg->get_value_fatal(tag, "quaternion", strs);
    quat_     = GET_RESOURCE(strs[0], const Eigen::VectorXd*);
    quat_cov_ = GET_RESOURCE(strs[1], const Eigen::MatrixXd*);

    cfg->get_value_fatal(tag, "linear_acc", strs);
    lin_acc_     = GET_RESOURCE(strs[0], const Eigen::VectorXd*);
    lin_acc_cov_ = GET_RESOURCE(strs[1], const Eigen::MatrixXd*);

    cfg->get_value_fatal(tag, "angular_vel", strs);
    ang_vel_     = GET_RESOURCE(strs[0], const Eigen::VectorXd*);
    ang_vel_cov_ = GET_RESOURCE(strs[1], const Eigen::MatrixXd*);
  }
};

struct _BodyTopology {
  double length;
  double height;
  double width;

  _BodyTopology(const MiiString& tag) {
    auto cfg = MiiCfgReader::instance();
    cfg->get_value_fatal(tag, "length", length);
    cfg->get_value_fatal(tag, "height", height);
    cfg->get_value_fatal(tag, "width" , width);
  }
};

DataBody::DataBody(const MiiString& _l)
  : Label(_l), imu_sensor_(nullptr),
    body_size_(nullptr) {
  ;
}

bool DataBody::init() {
  imu_sensor_ = new _ImuSensor(Label::make_label(getLabel(), "imu"));
  body_size_  = new _BodyTopology(Label::make_label(getLabel(), "topology"));
  // auto cfg = MiiCfgReader::instance();

  return true;
}

DataBody::~DataBody() {
  delete imu_sensor_;
  imu_sensor_ = nullptr;
}

double DataBody::body_length() const
{ return body_size_->length; }

double DataBody::body_height() const
{ return body_size_->height; }

double DataBody::body_width()  const
{ return body_size_->width; }

Eigen::VectorXd        DataBody::orientation()               const
{ return *imu_sensor_->quat_; }

const Eigen::VectorXd& DataBody::orientation_const_ref()     const
{ return *imu_sensor_->quat_; }

const Eigen::VectorXd* DataBody::orientation_const_pointer() const
{ return imu_sensor_->quat_; }

Eigen::MatrixXd        DataBody::orientation_covariance()               const
{ return *imu_sensor_->quat_cov_; }

const Eigen::MatrixXd& DataBody::orientation_covariance_const_ref()     const
{ return *imu_sensor_->quat_cov_; }

const Eigen::MatrixXd* DataBody::orientation_covariance_const_pointer() const
{ return imu_sensor_->quat_cov_; }

Eigen::VectorXd        DataBody::angular_velocity()               const
{ return *imu_sensor_->ang_vel_; }

const Eigen::VectorXd& DataBody::angular_velocity_const_ref()     const
{ return *imu_sensor_->ang_vel_; }

const Eigen::VectorXd* DataBody::angular_velocity_const_pointer() const
{ return imu_sensor_->ang_vel_; }

Eigen::MatrixXd        DataBody::angular_velocity_covariance()               const
{ return *imu_sensor_->ang_vel_cov_; }

const Eigen::MatrixXd& DataBody::angular_velocity_covariance_const_ref()     const
{ return *imu_sensor_->ang_vel_cov_; }

const Eigen::MatrixXd* DataBody::angular_velocity_covariance_const_pointer() const
{ return imu_sensor_->ang_vel_cov_; }

Eigen::VectorXd        DataBody::linear_acceleration()               const
{ return *imu_sensor_->lin_acc_; }

const Eigen::VectorXd& DataBody::linear_acceleration_const_ref()     const
{ return *imu_sensor_->lin_acc_; }

const Eigen::VectorXd* DataBody::linear_acceleration_const_pointer() const
{ return imu_sensor_->lin_acc_; }

Eigen::MatrixXd        DataBody::linear_acceleration_covariance()               const
{ return *imu_sensor_->lin_acc_cov_; }

const Eigen::MatrixXd& DataBody::linear_acceleration_covariance_const_ref()     const
{ return *imu_sensor_->lin_acc_cov_; }

const Eigen::MatrixXd* DataBody::linear_acceleration_covariance_const_pointer() const
{ return imu_sensor_->lin_acc_cov_; }

} /* namespace qr_control */
