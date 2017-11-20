#ifndef QR_CONTROL_FOOT_CONTACT_H_
#define QR_CONTROL_FOOT_CONTACT_H_

#include <cmath>
#include "qr_control/ultility.h"
#include <iostream>


enum ContactStatus {None=0, Deep};


namespace qr_control {

	class FootContact {
	private:

	    /////////////////////////////////////////////////
		int seq_size;
		bool verbose_;
		bool leg_on_ground;

		ContactStatus lf_contact_status;
		ContactStatus rf_contact_status;
		ContactStatus lb_contact_status;
		ContactStatus rb_contact_status;

		int lf_feature_one_threshold;
		int rf_feature_one_threshold;
		int lb_feature_one_threshold;
		int rb_feature_one_threshold;

		int lf_feature_two_threshold;
		int rf_feature_two_threshold;
		int lb_feature_two_threshold;
		int rb_feature_two_threshold;

		int lf_feature_three_threshold;
		int rf_feature_three_threshold;
		int lb_feature_three_threshold;
		int rb_feature_three_threshold;


		int lf_foot_force_arr[100];
		int rf_foot_force_arr[100];
		int lb_foot_force_arr[100];
		int rb_foot_force_arr[100];
		
		int sensorValueFirstAverage(int *data);
		int sensorValueLastAverage(int *data);

		ContactStatus featureOne(int legId);
		ContactStatus featureTwo(int legId);
		ContactStatus featureThree(int legId);

	public:     
		FootContact();

		void setSeqSize(int length);
		void setFeatureOneThreshold(int lf_threshold, int rf_threshold, int lb_threshold, int rb_threshold);
		void setFeatureTwoThreshold(int lf_threshold, int rf_threshold, int lb_threshold, int rb_threshold);
		void setFeatureThreeThreshold(int lf_threshold, int rf_threshold, int lb_threshold, int rb_threshold);
		void footForceDataUpdate(int lf_foot_force, int rf_foot_force, int lb_foot_force, int rb_foot_force);
		void tdSingleEventDetect(int legId);
		void tdSingleEventConditionDetect(int legId);
		void init();
		void clear();
		int getLastfootForceData(int legId);
		int getLastfootDiffForceData(int legId);
		void printThreshold();
		void printForceArr();
		void printForce();
		bool isLegOnGround();
		ContactStatus singleFootContactStatus(int legId);
		
	};
}
#endif

