#include "qr_control/foot_contact.h"

using namespace qr_control;


FootContact::FootContact()
{}

void FootContact::footForceDataUpdate(int lf_foot_force, int rf_foot_force, int lb_foot_force, int rb_foot_force)
{

	lf_foot_force_ = lf_foot_force;	
	rf_foot_force_ = rf_foot_force;	
	lb_foot_force_ = lb_foot_force;	
	rb_foot_force_ = rb_foot_force;	
}

void FootContact::setThreshold(int lf_threshold, int rf_threshold, int lb_threshold, int rb_threshold)
{
	lf_threshold_ = lf_threshold;
	rf_threshold_ = rf_threshold;
	lb_threshold_ = lb_threshold;
	rb_threshold_ = rb_threshold;
}
void FootContact::setUpperThreshold(int threshold)
{
	upper_thres_ = threshold;
}


void FootContact::setConst(int lf, int rf, int lb, int rb)
{
  lf_const_ = lf;
  rf_const_ = rf;
  lb_const_ = lb;
  rb_const_ = rb;
}

void FootContact::tdSingleEventDetect(int legId)
{
	switch(legId)
	{
		case LF:		
		  lf_contact_status = (lf_foot_force_ > lf_threshold_) ? Deep : None;		
		  break;	
		case RF:		
		  rf_contact_status = (rf_foot_force_ > rf_threshold_) ? Deep : None;	
		  break;	
		case LB:
		  lb_contact_status = (lb_foot_force_ > lb_threshold_) ? Deep : None;	
		  break;	
		case RB:	
		  rb_contact_status = (rb_foot_force_ > rb_threshold_) ? Deep : None;	
		  break;	
		default:break;	
	}	

	if(lf_contact_status | rf_contact_status | lb_contact_status | rb_contact_status)
	{
		leg_on_ground = true;
	}
}

bool FootContact::overDetect(int legId)
{
	switch(legId)
	{
		case LF:				
			return ((lf_foot_force_ > upper_thres_) ? true : false);			
		case RF:		
			return ((rf_foot_force_ > upper_thres_) ? true : false);
		case LB:
			return ((lb_foot_force_ > upper_thres_) ? true : false);		
		case RB:	
			return ((rb_foot_force_ > upper_thres_) ? true : false);		
		default:break;
	}	
	return false;
}
void FootContact::tdEventDetect()
{
  lf_contact_status = (lf_foot_force_ > lf_threshold_) ? Deep : None;			  
	rf_contact_status = (rf_foot_force_ > rf_threshold_) ? Deep : None;			 
	lb_contact_status = (lb_foot_force_ > lb_threshold_) ? Deep : None;			  
	rb_contact_status = (rb_foot_force_ > rb_threshold_) ? Deep : None;	
}

void FootContact::tdSingleEventConditionDetect(int legId)
{
	switch(legId)
	{
		case LF:				
			lf_contact_status = lf_contact_status ? Deep : ((lf_foot_force_ > lf_threshold_) ? Deep : None);			
			break;	
		case RF:		
			rf_contact_status = rf_contact_status ? Deep : ((rf_foot_force_ > rf_threshold_) ? Deep : None);		
			break;	
		case LB:
			lb_contact_status = lb_contact_status ? Deep : ((lb_foot_force_ > lb_threshold_) ? Deep : None);			
			break;	
		case RB:	
			rb_contact_status = rb_contact_status ? Deep : ((rb_foot_force_ > rb_threshold_) ? Deep : None);		
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

std::vector<bool> FootContact::contactStatus()
{
  std::vector<bool> result(4,0);
  result[LF] = lf_contact_status;
  result[RF] = rf_contact_status;
  result[LB] = lb_contact_status;
  result[RB] = rb_contact_status;
  
  return result;
}

float FootContact::getForceData(int legId)
{
	switch(legId)
	{
		case LF:
			return lf_foot_force_;
		case RF:
			return rf_foot_force_;
		case LB:
			return lb_foot_force_;
		case RB:
			return rb_foot_force_;
	}	
}

float FootContact::getForceConst(int legId)
{
	switch(legId)
	{
		case LF:
			return lf_const_;
		case RF:
			return rf_const_;
		case LB:
			return lb_const_;
		case RB:
			return rb_const_;
	}	
}

void FootContact::printThreshold()
{
	std::cout<<"Class FootContact: printThreshold"<<std::endl;
	std::cout<<"lf:"<<lf_threshold_<<" ";
	std::cout<<"rf:"<<rf_threshold_<<" ";
	std::cout<<"lb:"<<lb_threshold_<<" ";
	std::cout<<"rb:"<<rb_threshold_<<" "<<std::endl;
}
		
void FootContact::printConst()
{
	std::cout<<"Class FootContact: printConst"<<std::endl;
	std::cout<<"lf:"<<lf_const_<<" ";
	std::cout<<"rf:"<<rf_const_<<" ";
	std::cout<<"lb:"<<lb_const_<<" ";
	std::cout<<"rb:"<<rb_const_<<" "<<std::endl;
}

void FootContact::printForce()
{
	std::cout<<"Class FootContact: printForce"<<" ";
	std::cout<<"lf:"<<lf_foot_force_<<" ";
	std::cout<<"rf:"<<rf_foot_force_<<" ";
	std::cout<<"lb:"<<lb_foot_force_<<" ";
	std::cout<<"rb:"<<rb_foot_force_<<std::endl;
}

void FootContact::printForce(int legId)
{
	std::cout<<"Class FootContact: printForce"<<" ";
	
	switch(legId)
	{
		case LF:
			std::cout<<"lf:"<<lf_foot_force_<<std::endl;
			break;
		case RF:
			std::cout<<"rf:"<<rf_foot_force_<<std::endl;
			break;
		case LB:
			std::cout<<"lb:"<<lb_foot_force_<<std::endl;
			break;
		case RB:
			std::cout<<"rb:"<<rb_foot_force_<<std::endl;
			break;
	}		
}

void FootContact::printContactStatus()
{
	std::cout<<"Class FootContact: printContactStatus"<<" ";
	std::cout<<"lf:"<<lf_contact_status<<" ";
	std::cout<<"rf:"<<rf_contact_status<<" ";
	std::cout<<"lb:"<<lb_contact_status<<" ";
	std::cout<<"rb:"<<rb_contact_status<<std::endl;
}
