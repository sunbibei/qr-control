/*
 * leg_robot.h
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_LEG_ROBOT_H_
#define INCLUDE_ROBOT_LEG_ROBOT_H_

// #include "foudantion/label.h"
#include "leg/robot_leg.h"
#include "body/robot_body.h"

#include <Eigen/Dense>

namespace qr_control {
/*!
 * @brief Any sub-class inherit from LegRobot, is need be implemented as a singleton,
 *        and pass a _tag(the tag of configure file) as the parameter of constructor.
 *        the format of configure file as follow:
 *        <LegRobot legs="leg_label_0 leg_label_1 leg_label_2 leg_label_3" body="body_label" />
 *        This class is similar to a manager of robot's resources, it avoid the fact that each
 *        gait need the same configure to get the interfaces of leg and body from LabelTable,
 *        and it is designed as a high-level information provider, such as stability margin,
 *        the response of emergency, the origin of camera(not now), the odometer, and so on.
 */
class LegRobot {
protected:
  /*!
   * @brief The tag of configure file.
   */
	LegRobot(const MiiString& _tag);
	virtual ~LegRobot();

public:
	/*!
	 * @brief Calculate the stability margin of the current robot.
	 */
	virtual double stability_margin() const = 0;

///! The getter for the interfaces of robot
public:
	/*!
	 * @brief The interface for robot leg.
	 */
	virtual RobotLeg* robot_leg(LegType);
	/*!
	 * @brief The interface for robot body,
	 */
	virtual RobotBody* robot_body();

protected:
	RobotLeg*  leg_ifaces_[LegType::N_LEGS];
	RobotBody* body_iface_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_LEG_ROBOT_H_ */
