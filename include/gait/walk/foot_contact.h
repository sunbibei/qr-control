#ifndef QR_CONTROL_FOOT_CONTACT_H_
#define QR_CONTROL_FOOT_CONTACT_H_

#include <cmath>
#include <iostream>

#include "ultility.h"

enum ContactStatus {None=0, Deep};


namespace qr_control {

	class FootContact {
	private:

	    /////////////////////////////////////////////////
		bool verbose_;
		bool leg_on_ground;

		ContactStatus lf_contact_status;
		ContactStatus rf_contact_status;
		ContactStatus lb_contact_status;
		ContactStatus rb_contact_status;

		int lf_threshold_;
		int rf_threshold_;
		int lb_threshold_;
		int rb_threshold_;	
		int upper_thres_;

		float lf_foot_force_;		
		float rf_foot_force_;	
		float lb_foot_force_;	
		float rb_foot_force_;	

		float lf_const_;		
		float rf_const_;	
		float lb_const_;	
		float rb_const_;	

	public:     
		FootContact();

		void setThreshold(int lf_threshold, int rf_threshold, int lb_threshold, int rb_threshold);
		void setUpperThreshold(int threshold);
		void setConst(int lf, int rf, int lb, int rb);
		void footForceDataUpdate(int lf_foot_force, int rf_foot_force, int lb_foot_force, int rb_foot_force);
		void tdSingleEventDetect(int legId);
		void tdEventDetect();
		bool overDetect(int legId);
		void tdSingleEventConditionDetect(int legId);
		void init();
		void clear();
		float getForceData(int legId);
		float getForceConst(int legId);
		void printThreshold();
		void printForce();
		void printConst();
		void printContactStatus();

		bool isLegOnGround();
		ContactStatus singleFootContactStatus(int legId);
		std::vector<bool> contactStatus();
		
	};
}
#endif

