/*
 * mii_control.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: bibei
 */

#include <mii_control.h>

#include <foundation/cfg_reader.h>
#include <foundation/auto_instanceor.h>
#include <system/platform/thread/threadpool.h>

#include "gait/gait_manager.h"

#include <thread>

namespace qr_control {

#define MII_CTRL ("mii-control")

SINGLETON_IMPL_NO_CREATE(MiiControl)

void __auto_inst(const MiiString& _p, const MiiString& _type) {
  if (!AutoInstanceor::instance()->make_instance(_p, _type))
    LOG_ERROR << "Create instance(" << _type << " " << _p << ") fail!";
}

MiiControl* MiiControl::create_instance(const MiiString& prefix) {
  if (nullptr != instance_) {
    LOG_WARNING << "This method 'create_instance()' is called twice.";
  } else {
    instance_ = new MiiControl(prefix);
  }
  return instance_;
}

MiiControl::MiiControl(const MiiString& _prefix)
  : prefix_tag_(_prefix), alive_(true),
    tick_interval_(20) {
}

MiiControl::~MiiControl() {

}

bool MiiControl::init() {
  create_system_instance();

  auto cfg = MiiCfgReader::instance();
  if (nullptr == cfg) {
    LOG_FATAL << "The MiiCfgReader::create_instance(MiiStringConstRef) "
        << "method must to be called by subclass before MiiRobot::init()";
  }
  double hz = 50;
  cfg->get_value(prefix_tag_, "frequency", hz);
  tick_interval_ = std::chrono::milliseconds(int(1000/hz));

  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  cfg->regAttrCb("auto_inst", __auto_inst, prefix_tag_);
  // Just for debug
  LOG_DEBUG << "Auto instance has finished. The results list as follow:";
  Label::printfEveryInstance();

  return true;
}

bool MiiControl::start() {
  //  TODO
  // middleware::ThreadPool::instance()->add(MII_CTRL, &MiiControl::tick, this);
  return true;
}

void MiiControl::halt() {
  ; // Nothing to do here.
}

void MiiControl::activate(const MiiString& _n) {
  GaitManager::instance()->activate(_n);
}

void MiiControl::create_system_instance() {
  if (nullptr == GaitManager::create_instance())
    LOG_FATAL << "Create the singleton 'GaitManager' has failed.";
}

void MiiControl::tick() {
  TIMER_INIT

  auto manager = GaitManager::instance();
  while (alive_) {
    manager->tick();

    TIMER_CONTROL(tick_interval_)
  }
}

} /* namespace qr_control */
