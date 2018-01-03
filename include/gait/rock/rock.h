/*
 * rock.h
 *
 *  Created on: Jan 3, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_ROCK_ROCK_H_
#define INCLUDE_GAIT_ROCK_ROCK_H_

#include "../gait_base.h"

namespace qr_control {

enum RockState {
	RK_ONE = 0,
	RK_TWO,
};

class Rock: public GaitBase {
public:
	Rock();
	virtual bool init() override;

	virtual ~Rock();

protected:
	RockState                current_state_;
	StateMachine<RockState>* state_machine_;

protected:
	virtual bool starting() override;
	virtual void stopping() override;

	virtual void checkState() override;
	virtual S
	virtual void prev_tick() override;
	virtual void post_tick() override;

protected:
	class QrLeg*  leg_ifaces_[LegType::N_LEGS];
	class QrBody* body_iface_;
	class RockParams* params_;

};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_ROCK_ROCK_H_ */
