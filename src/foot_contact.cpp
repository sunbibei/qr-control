#include "qr_control/foot_contact.h"

using namespace qr_control;


FootContact::FootContact()
{}

void FootContact::setSeqSize(int length)//1<length<100, even. length is to decide how many data will be used to detect TD events
{
	seq_size = length;
}

void FootContact::footForceDataUpdate(int lf_foot_force, int rf_foot_force, int lb_foot_force, int rb_foot_force)
{
	for(int i=1; i<seq_size; i++)
	{
		lf_foot_force_arr[i-1] = lf_foot_force_arr[i];
		rf_foot_force_arr[i-1] = rf_foot_force_arr[i];
		lb_foot_force_arr[i-1] = lb_foot_force_arr[i];
		rb_foot_force_arr[i-1] = rb_foot_force_arr[i];
	}

	lf_foot_force_arr[seq_size-1] = lf_foot_force;	
	rf_foot_force_arr[seq_size-1] = rf_foot_force;	
	lb_foot_force_arr[seq_size-1] = lb_foot_force;	
	rb_foot_force_arr[seq_size-1] = rb_foot_force;	
}

int FootContact::sensorValueFirstAverage(int *data)
{
	int tmp = 0;
	for(int i=0; i<seq_size/2; i++)
	{
		tmp += data[i];
	}
	return (tmp/seq_size/2);
}

int FootContact::sensorValueLastAverage(int *data)
{
	int tmp = 0;
	for(int i=seq_size-1; i>=seq_size/2; i--)
	{
		tmp += data[i];
	}
	return (tmp/seq_size/2);
}

ContactStatus FootContact::featureOne(int legId)
{
	int first = 0;
	int last = 0;

	switch(legId)
	{
		case LF:
		first = sensorValueFirstAverage(lf_foot_force_arr);
		last = sensorValueLastAverage(lf_foot_force_arr);
		return (((first-last) > lf_feature_one_threshold) ? Deep : None);			
		case RF:
		first = sensorValueFirstAverage(rf_foot_force_arr);
		last = sensorValueLastAverage(rf_foot_force_arr);
		return (((first-last) > rf_feature_one_threshold) ? Deep : None);		
		case LB:
		first = sensorValueFirstAverage(lb_foot_force_arr);
		last = sensorValueLastAverage(lb_foot_force_arr);
		return (((first-last) > lb_feature_one_threshold) ? Deep : None);		
		case RB:
		first = sensorValueFirstAverage(rb_foot_force_arr);
		last = sensorValueLastAverage(rb_foot_force_arr);
		return (((first-last) > rb_feature_one_threshold) ? Deep : None);			
	}	
}

ContactStatus FootContact::featureTwo(int legId)
{
	switch(legId)
	{
		case LF:
		return ((fabs(lf_foot_force_arr[seq_size-2] - lf_foot_force_arr[seq_size-1]) > lf_feature_two_threshold) ? Deep : None);	
		case RF:
		return ((fabs(rf_foot_force_arr[seq_size-2] - rf_foot_force_arr[seq_size-1]) > rf_feature_two_threshold) ? Deep : None);
		case LB:
		return ((fabs(lb_foot_force_arr[seq_size-2] - lb_foot_force_arr[seq_size-1]) > lb_feature_two_threshold) ? Deep : None);
		case RB:
		return ((fabs(rb_foot_force_arr[seq_size-2] - rb_foot_force_arr[seq_size-1]) > rb_feature_two_threshold) ? Deep : None);	
	}	
}

ContactStatus FootContact::featureThree(int legId)
{
	switch(legId)
	{
		case LF:
		return (((lf_foot_force_arr[seq_size-2] - lf_foot_force_arr[seq_size-1] > lf_feature_three_threshold)
			&& (lf_foot_force_arr[seq_size-3] - lf_foot_force_arr[seq_size-2] > lf_feature_three_threshold)) ? Deep : None);	
		case RF:
		return (((rf_foot_force_arr[seq_size-2] - rf_foot_force_arr[seq_size-1] > rf_feature_three_threshold)
			&& (rf_foot_force_arr[seq_size-3] - rf_foot_force_arr[seq_size-2] > rf_feature_three_threshold)) ? Deep : None);
		case LB:
		return (((lb_foot_force_arr[seq_size-2] - lb_foot_force_arr[seq_size-1] > lb_feature_three_threshold)
			&& (lb_foot_force_arr[seq_size-3] - lb_foot_force_arr[seq_size-2] > lb_feature_three_threshold)) ? Deep : None);
		case RB:
		return (((rb_foot_force_arr[seq_size-2] - rb_foot_force_arr[seq_size-1] > rb_feature_three_threshold)
			&& (rb_foot_force_arr[seq_size-3] - rb_foot_force_arr[seq_size-2] > rb_feature_three_threshold)) ? Deep : None);	
	}	
}

void FootContact::setFeatureOneThreshold(int lf_threshold, int rf_threshold, int lb_threshold, int rb_threshold)
{
	lf_feature_one_threshold = lf_threshold;
	rf_feature_one_threshold = rf_threshold;
	lb_feature_one_threshold = lb_threshold;
	rb_feature_one_threshold = rb_threshold;
}

void FootContact::setFeatureTwoThreshold(int lf_threshold, int rf_threshold, int lb_threshold, int rb_threshold)
{
	lf_feature_two_threshold = lf_threshold;
	rf_feature_two_threshold = rf_threshold;
	lb_feature_two_threshold = lb_threshold;
	rb_feature_two_threshold = rb_threshold;
}

void FootContact::setFeatureThreeThreshold(int lf_threshold, int rf_threshold, int lb_threshold, int rb_threshold)
{
	lf_feature_three_threshold = lf_threshold;
	rf_feature_three_threshold = rf_threshold;
	lb_feature_three_threshold = lb_threshold;
	rb_feature_three_threshold = rb_threshold;
}

void FootContact::tdSingleEventDetect(int legId)
{
	switch(legId)
	{
		case LF:				
			lf_contact_status = (featureOne(legId) | featureTwo(legId) | featureThree(legId)) ? Deep : None;			
			break;	
		case RF:		
			rf_contact_status = (featureOne(legId) | featureTwo(legId) | featureThree(legId)) ? Deep : None;	
			break;	
		case LB:
			lb_contact_status = (featureOne(legId) | featureTwo(legId) | featureThree(legId)) ? Deep : None;			
			break;	
		case RB:	
			rb_contact_status = (featureOne(legId) | featureTwo(legId) | featureThree(legId)) ? Deep : None;		
			break;
		default:break;
	}

	if(lf_contact_status | rf_contact_status | lb_contact_status | rb_contact_status)
	{
		leg_on_ground = true;
	}
}

void FootContact::tdSingleEventConditionDetect(int legId)
{
	switch(legId)
	{
		case LF:				
			lf_contact_status = lf_contact_status ? Deep : ((featureOne(legId) | featureTwo(legId) | featureThree(legId)) ? Deep : None);			
			break;	
		case RF:		
			rf_contact_status = rf_contact_status ? Deep : ((featureOne(legId) | featureTwo(legId) | featureThree(legId)) ? Deep : None);		
			break;	
		case LB:
			lb_contact_status = lb_contact_status ? Deep : ((featureOne(legId) | featureTwo(legId) | featureThree(legId)) ? Deep : None);			
			break;	
		case RB:	
			rb_contact_status = rb_contact_status ? Deep : ((featureOne(legId) | featureTwo(legId) | featureThree(legId)) ? Deep : None);		
			break;
		default:break;
	}	

	if(lf_contact_status | rf_contact_status | lb_contact_status | rb_contact_status)
	{
		leg_on_ground = true;
	}
}

void FootContact::clear()
{
	lf_contact_status = None;
	rf_contact_status = None;
	lb_contact_status = None;
	rb_contact_status = None;
	leg_on_ground = false;
}

void FootContact::init()
{
	lf_contact_status = None;
	rf_contact_status = None;
	lb_contact_status = None;
	rb_contact_status = None;
	leg_on_ground = false;
}

bool FootContact::isLegOnGround()
{
	return leg_on_ground;
}

ContactStatus FootContact::singleFootContactStatus(int legId)
{
	switch(legId)
	{
		case LF:
			return lf_contact_status;
		case RF:
			return rf_contact_status;
		case LB:
			return lb_contact_status;
		case RB:
			return rb_contact_status;
	}	
}


int FootContact::getLastfootForceData(int legId)
{
	switch(legId)
	{
		case LF:
			return lf_foot_force_arr[seq_size-1];
		case RF:
			return rf_foot_force_arr[seq_size-1];
		case LB:
			return lb_foot_force_arr[seq_size-1];
		case RB:
			return rb_foot_force_arr[seq_size-1];
	}	
}

int FootContact::getLastfootDiffForceData(int legId)
{
	switch(legId)
	{
		case LF:
			return lf_foot_force_arr[seq_size-1] - lf_foot_force_arr[seq_size-2];
		case RF:
			return rf_foot_force_arr[seq_size-1] - rf_foot_force_arr[seq_size-2];
		case LB:
			return lb_foot_force_arr[seq_size-1] - lb_foot_force_arr[seq_size-2];
		case RB:
			return rb_foot_force_arr[seq_size-1] - rb_foot_force_arr[seq_size-2];
	}	
}

void FootContact::printThreshold()
{
	std::cout<<"Class FootContact:"<<std::endl;
	std::cout<<"lf:"<<lf_feature_one_threshold<<" ";
	std::cout<<"rf:"<<rf_feature_one_threshold<<" ";
	std::cout<<"lb:"<<lb_feature_one_threshold<<" ";
	std::cout<<"rb"<<rb_feature_one_threshold<<" "<<std::endl;
	std::cout<<"lf:"<<lf_feature_two_threshold<<" ";
	std::cout<<"rf:"<<rf_feature_two_threshold<<" ";
	std::cout<<"lb:"<<lb_feature_two_threshold<<" ";
	std::cout<<"rb:"<<rb_feature_two_threshold<<" "<<std::endl;
}
		


void FootContact::printForce()
{
	std::cout<<"Class FootContact:"<<" ";
	std::cout<<"lf:"<<lf_foot_force_arr[seq_size-1]<<" ";
	std::cout<<"rf:"<<rf_foot_force_arr[seq_size-1]<<" ";
	std::cout<<"lb:"<<lb_foot_force_arr[seq_size-1]<<" ";
	std::cout<<"rb:"<<rb_foot_force_arr[seq_size-1]<<std::endl;
}
void FootContact::printForceArr()
{
	std::cout<<"Class FootContact:"<<std::endl;
	std::cout<<"lf:";
	for(int i=0; i<seq_size; i++)
	{
		std::cout<<lf_foot_force_arr[i]<<" ";	
	}
	std::cout<<std::endl;

	std::cout<<"rf:";
	for(int i=0; i<seq_size; i++)
	{
		std::cout<<rf_foot_force_arr[i]<<" ";	
	}
	std::cout<<std::endl;

	std::cout<<"lb:";
	for(int i=0; i<seq_size; i++)
	{
		std::cout<<lb_foot_force_arr[i]<<" ";	
	}
	std::cout<<std::endl;

	std::cout<<"rb:";
	for(int i=0; i<seq_size; i++)
	{
		std::cout<<rb_foot_force_arr[i]<<" ";	
	}
	std::cout<<std::endl;
}




