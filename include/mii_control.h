/*
 * mii_control.h
 *
 *  Created on: Nov 30, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_MII_CONTROL_H_
#define INCLUDE_MII_CONTROL_H_

#include <chrono>
#include <foundation/utf.h>

namespace qr_control {

class MiiControl {
  SINGLETON_DECLARE(MiiControl, const MiiString&)

public:
  /*!
   * @brief This method will completed the initialization for MII Control.
   */
  virtual bool init();

  /*!
   * @brief Starting to work
   */
  virtual bool start();
  /*!
   * @brief halt
   */
  virtual void halt();

  /*!
   * @brief Switch to the different gait mode. This action is sync, the method
   *        is not change the gait right now, but add the gait mode into the
   *        ready queue, and switch will be completed in a appropriate opportunity.
   */
  virtual void activate(const MiiString&);

protected:
  /*!
   * @brief The all of singleton has created.
   */
  virtual void create_system_instance();

  virtual void tick();

protected:
  MiiString prefix_tag_;

  bool alive_;
  std::chrono::milliseconds tick_interval_;

};

} /* namespace qr_control */

#endif /* INCLUDE_MII_CONTROL_H_ */
