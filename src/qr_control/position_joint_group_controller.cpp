#include "qr_control/position_joint_group_controller.h"

namespace qr_control
{
	using namespace hardware_interface;

	PositionJointGroupController::PositionJointGroupController()
	{
		Loop_Count = 0;
	}

	PositionJointGroupController::~PositionJointGroupController()
	{
		delete foot_contact;
		delete swing;
		delete math;
	}	

/**************************************************************************
   Description: Update Angles
**************************************************************************/
	void PositionJointGroupController::update_angle()
	{
		Angle_ptr->lb.hip  = joints_[0].getPosition();
		Angle_ptr->lb.knee = joints_[1].getPosition();
		Angle_ptr->lb.pitch= joints_[2].getPosition();
		Angle_ptr->lf.hip  = joints_[3].getPosition();
		Angle_ptr->lf.knee = joints_[4].getPosition();
		Angle_ptr->lf.pitch= joints_[5].getPosition();

		Angle_ptr->rb.hip  = joints_[6].getPosition();
		Angle_ptr->rb.knee = joints_[7].getPosition();
		Angle_ptr->rb.pitch= joints_[8].getPosition();
		Angle_ptr->rf.hip  = joints_[9].getPosition();
		Angle_ptr->rf.knee = joints_[10].getPosition();
		Angle_ptr->rf.pitch= joints_[11].getPosition();
	}


/**************************************************************************
   Description: initializition from robot_description
**************************************************************************/
	bool PositionJointGroupController::init(hardware_interface::RobotHW* robot, ros::NodeHandle &n)
	{
		urdf::Model urdf;
		if(!urdf.initParam("robot_description"))
		{
			ROS_ERROR("Failed to parse urdf file");
			return false;
		}
		n_joints_ = 0;
		PositionJointInterface* iface = robot->get<PositionJointInterface>();
		while(true) 
		{
			std::string joint_name;
			std::string param_name = std::string("joint_" + std::to_string(n_joints_));
			if (ros::param::get(param_name.c_str(), joint_name)) 
			{
				std::cout << "Get Joint Name: " << joint_name << std::endl;
				joints_.push_back(iface->getHandle(joint_name));
			} 
			else 
			{
				break;
			}
			++n_joints_;
		}

		int n_tds = 0;
		ForceTorqueSensorInterface* force = robot->get<ForceTorqueSensorInterface>();
		while(true) 
		{
			std::string joint_name;
			std::string param_name = std::string("touchdown_" + std::to_string(n_tds));
			if (ros::param::get(param_name.c_str(), joint_name)) 
			{
				std::cout << "Get Touchdown Name: " << joint_name << std::endl;
				td_handles_.push_back(force->getHandle(joint_name));
			} 
			else 
			{
				break;
			}
			++n_tds;
		}	

		for(int i=0; i<n_joints_; i++)
		{
			commands.push_back(0);
		}
	
		joint_state_publisher_.reset( new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(n, "/dragon/joint_commands", 10));

		//foot contact process
		foot_contact = new FootContact();
		foot_contact->setSeqSize(4);
		foot_contact->setFeatureOneThreshold(50, 50, 50, 50);
		foot_contact->setFeatureTwoThreshold(100, 100, 100, 100);
		foot_contact->setFeatureThreeThreshold(50,50,50,50);
		foot_contact->init();

		swing = new Swing();
		swing->init();

		math = new Math();
		math->init();

		gesture = new Gesture();
		gesture->init(robot);

		if(n_joints_ == Joint_Num)
		{
			std::cout<<"System Init Succeed!"<<std::endl;
		}
		return true;
	}

/**************************************************************************
   Description: start the controller
**************************************************************************/
	void PositionJointGroupController::starting(const ros::Time& time)
	{
			foot_contact->printThreshold();
	}

/**************************************************************************
   Description: design state meachine: Adjust CoG <---> Switch Swing Leg
**************************************************************************/
void PositionJointGroupController::update(const ros::Time& time, const ros::Duration& period)
{
	gesture->updateImuData();

	foot_contact->footForceDataUpdate(*(td_handles_[0].getForce()), *(td_handles_[1].getForce()), *(td_handles_[2].getForce()), *(td_handles_[3].getForce()));
	// foot_contact->printForceArr();
	
  _Position innerheart;
  
	switch (Time_Order)
	{
	case 0: //init gesture
	pose_init();			
	flow_control(Time_Order);
	break;

	case 1://		
	foot_contact->printForce();
	gesture->imuCalibration();
	// hardware_delay_test();
	// if(Loop_Count<5)
	// {
	// 	foot_sensor_const = foot_contact->getLastfootForceData(LB);
	// }
	// else
	// {
	// 	stay_touch(foot_contact->getLastfootForceData(LB));
	// }
	flow_control(Time_Order);
	break;

	case 2:
	gesture->updateShoulderPos();		
	cog_adj();	  	
	flow_control(Time_Order);
	break;

	case 3://move cog	
	
	height_recovery((Leg_Order<2)?Leg_Order+2:3-Leg_Order, 1);
	if(Leg_Order>1)
	{
		height_recovery(5-Leg_Order, 1);
	}

	flow_control(Time_Order);
	break;	

	case 4://swing leg	
	swing_control();
	flow_control(Time_Order);
	break;	
	case 5:
	// if(Leg_Order==3)
	// {
	// 	std::cin>>Leg_Order;
	// }
	height_recovery(5-Leg_Order, -1);
	flow_control(Time_Order);

	default:
	break;
}

Loop_Count++;
commands = vec_assign(Angle_ptr);

// std::cout<<"Joint Commands:";
for(int i=0; i<n_joints_; i++)
{
	// std::cout<<commands[i]<<" ";
	joints_[i].setCommand(commands[i]);
}
// std::cout<<std::endl;

if(joint_state_publisher_ && joint_state_publisher_->trylock())
{
	joint_state_publisher_->msg_.data.clear();
	for(int i=0; i<n_joints_; i++)
	{
		joint_state_publisher_->msg_.data.push_back(commands[i]);
	}
	joint_state_publisher_->unlockAndPublish();
}
}

void PositionJointGroupController::hardware_delay_test()
{
	_Angle_Leg A;
	_Position P;	

	if(Loop_Count<=1)
	{
		std::cout<<"Before:"<<Angle_ptr->lf.hip<<std::endl;
		Pos_ptr->rb.z = -Stance_Height;
		Pos_ptr->lb.z = -Stance_Height;
		Pos_ptr->rf.z = -Stance_Height;
		Pos_ptr->lf.z = -Stance_Height;

		Pos_ptr->lf.y = 10;
		Pos_ptr->rf.y = 10;
		Pos_ptr->lb.y = 10;
		Pos_ptr->rb.y = 10;

		Pos_ptr->lf.x = 10;
		Pos_ptr->rf.x = 10;
		Pos_ptr->lb.x = 10;
		Pos_ptr->rb.x = 10;	
		reverse_kinematics();	
		std::cout<<"After:"<<Angle_ptr->lf.hip<<std::endl;
		std::cout<<"Start time "<<Loop_Count<<std::endl;
	}
	else
	{			
		A.hip  = joints_[3].getPosition();
		A.knee = joints_[4].getPosition();
		A.pitch= joints_[5].getPosition();
		P = math->cal_formula(A);
		std::cout<<"Loop_Count "<<Loop_Count<<" "<<P.x<<" "<<P.y<<" "<<P.z<<std::endl;		
	}	

}

void PositionJointGroupController::stay_touch(int sensor)
{
	// float adj = 0, pos_alpha = 0.0005, neg_alpha = -0.0005;//
	float adj = 0, pos_alpha = 0.002, neg_alpha = -0.002;//RB,LB

	std::cout<<"LB foot sensor value:"<<foot_contact->getLastfootForceData(LB)<<std::endl;

  // float stable_top = foot_sensor_const + 200;
  // float stable_floor = foot_sensor_const - 200;
  // float stable_top = foot_sensor_const + 50;//RB
  // float stable_floor = foot_sensor_const - 30;
  float stable_top = foot_sensor_const + 50;
  float stable_floor = foot_sensor_const - 30;
	std::cout<<"foot_sensor_const:"<<foot_sensor_const<<std::endl;

	if(stable_floor<sensor && sensor<stable_top)
	{
		std::cout<<"foot force good enough!"<<std::endl;
	}	
	if(sensor>stable_top)
	{
		adj = neg_alpha * fabs(stable_top - sensor);
	}
	if(sensor<stable_floor)
	{
		adj = pos_alpha * fabs(stable_floor - sensor);
	}
	std::cout<<"adj:"<<adj<<std::endl;

	Pos_ptr->lb.z += adj;
	reverse_kinematics();
}

void PositionJointGroupController::height_recovery(int legId, int sgn)
{
	double err = 0.1*sgn;
	if(Height_recovery_count)
	{
		switch (legId)
		{
			case LF:	
			// Height.lf = Height.lf + err;
			Pos_ptr->lf.z = Pos_ptr->lf.z + err;
			Angle_ptr->lf = math->cal_kinematics(Pos_ptr->lf,-1);
			break;
			case RF:
			// Height.rf = Height.rf + err;
			Pos_ptr->rf.z = Pos_ptr->rf.z + err;
			Angle_ptr->rf = math->cal_kinematics(Pos_ptr->rf,-1);
			break;
			case LB:			
			// Height.lb = Height.lb + err;			
			Pos_ptr->lb.z = Pos_ptr->lb.z + err;
			Angle_ptr->lb = math->cal_kinematics(Pos_ptr->lb,1);			
			break;
			case RB:
			// Height.rb = Height.rb + err;
			Pos_ptr->rb.z = Pos_ptr->rb.z + err;
			Angle_ptr->rb = math->cal_kinematics(Pos_ptr->rb,1);
			break;
			default:break;
		}
	}
}

void PositionJointGroupController::updownfuck()
{
	Pos_ptr->lb.y = Pos_ptr->lf.y = 0;
	Pos_ptr->rb.y = Pos_ptr->rf.y = 0;
	Pos_ptr->rf.x = Pos_ptr->lf.x = 0;
	Pos_ptr->rb.x = Pos_ptr->lb.x = 0;
	int add = 0;
  if(Loop_Count<50)//up
  {
  	add = get_adj_pos(10, Loop_Count, 50);
  	Pos_ptr->lf.z = -45 + add;
  	Pos_ptr->rf.z = -45 + add;
  	Pos_ptr->lb.z = -45 + add;
  	Pos_ptr->rb.z = -45 + add;
  }
  else if(Loop_Count<100)//down
  {
  	add = get_adj_pos(10, Loop_Count-50, 50);
  	Pos_ptr->lf.z = -45 + 5 - add;
  	Pos_ptr->rf.z = -45 + 5 - add;
  	Pos_ptr->lb.z = -45 + 5 - add;
  	Pos_ptr->rb.z = -45 + 5 - add;
  }
  else
  {
  	Loop_Count = 0;
  }
        // std::cout<<"foot pos"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
  reverse_kinematics();
}

void PositionJointGroupController::flow_control(int timeorder)
{
	switch(timeorder)
	{
		case 0:
		if(Loop_Count>=30)
		{
			Loop_Count = 0;
			std::cout<<"Robot has been initialized! Please input 1 to continue:";
			// std::cin>>Time_Order;	
			Time_Order=1;
		}
		break;		

		case 1:
		if(Loop_Count>=30)
		{
			Loop_Count = 0;
			std::cout<<"Imu data calibration done!"<<std::endl;
			Time_Order=2;
		}
		break;	

		case 2:
		if(Loop_Count>=Stance_Num)
		{
			Loop_Count = 0;			
			std::cout<<"Robot shift CoG done! Next swing leg: "<<Leg_Order<<std::endl;
			Time_Order=3;	
		}							
		break;	

		case 3:
		if(Loop_Count>=Height_recovery_count)
		{			
			Loop_Count = 0;
			Height_recovery_count = 0;			
			std::cout<<"Robot height has been recovery!"<<std::endl;
			Time_Order = 4;	
			assign_next_foot();
		}
		break;

		case 4:
		if(foot_contact->isLegOnGround())
		if(Loop_Count>=Swing_Num)
		{
			Loop_Count = 0;
			foot_contact->clear();

			if(Leg_Order<2)
			{
				Time_Order = 5;	
			}
			else
			{
				Time_Order = 2;	
			}
			Leg_Order = (Leg_Order>1)?Leg_Order-2:3-Leg_Order;						
			std::cout<<"Robot "<<Leg_Order<<" Leg swing done! Next shift CoG"<<std::endl;				
		}					
		break;

		case 5:
		if(Loop_Count>=Height_recovery_count)
		{
			Loop_Count = 0;						
			std::cout<<"Adjust foot position to avoid hanging!"<<std::endl;
			Time_Order=2;	
		}
		break;

		default:break;
	}
}

/**************************************************************************
   Description: Ensure the foot is on ground by recursive decreasing height
   Input: lf,rf,lb,rb corresponding to 0,1,2,3
**************************************************************************/
void PositionJointGroupController::on_ground_control(int legId)
{
	double err = 0.1;
	switch (legId)
	{
		case LF:	
			// Height.lf = Height.lf + err;
		Pos_ptr->lf.z = Pos_ptr->lf.z - err;
		Angle_ptr->lf = math->cal_kinematics(Pos_ptr->lf,-1);
		break;
		case RF:
			// Height.rf = Height.rf + err;
		Pos_ptr->rf.z = Pos_ptr->rf.z - err;
		Angle_ptr->rf = math->cal_kinematics(Pos_ptr->rf,-1);
		break;
		case LB:
			// Height.lb = Height.lb + err;
		Pos_ptr->lb.z = Pos_ptr->lb.z - err;
		Angle_ptr->lb = math->cal_kinematics(Pos_ptr->lb,1);
		break;
		case RB:
			// Height.rb = Height.rb + err;
		Pos_ptr->rb.z = Pos_ptr->rb.z - err;
		Angle_ptr->rb = math->cal_kinematics(Pos_ptr->rb,1);
		break;
		default:break;
	}
}

/**************************************************************************
   Description: Initialization. Foot position is relative to its corresponding shoulder
**************************************************************************/
void PositionJointGroupController::pose_init()
{	
	Pos_ptr->rb.z = -Stance_Height;
	Pos_ptr->lb.z = -Stance_Height;
	Pos_ptr->rf.z = -Stance_Height;
	Pos_ptr->lf.z = -Stance_Height;

	Pos_ptr->lf.y = 0;
	Pos_ptr->rf.y = 0;
	Pos_ptr->lb.y = 0;
	Pos_ptr->rb.y = 0;

	Pos_ptr->lf.x = 0;
	Pos_ptr->rf.x = 0;
	Pos_ptr->lb.x = 0;
	Pos_ptr->rb.x = 0;	
	reverse_kinematics();	
}


void PositionJointGroupController::forward_kinematics()
{
	Pos_ptr->lf = math->cal_formula(Angle_ptr->lf);
	Pos_ptr->rf = math->cal_formula(Angle_ptr->rf);
	Pos_ptr->lb = math->cal_formula(Angle_ptr->lb);
	Pos_ptr->rb = math->cal_formula(Angle_ptr->rb);
}


void PositionJointGroupController::reverse_kinematics()
{
	Angle_ptr->lf = math->cal_kinematics(Pos_ptr->lf,-1);
	Angle_ptr->rf = math->cal_kinematics(Pos_ptr->rf,-1);
	Angle_ptr->lb = math->cal_kinematics(Pos_ptr->lb, 1);
	Angle_ptr->rb = math->cal_kinematics(Pos_ptr->rb, 1);
}

_Position PositionJointGroupController::innerTriangle(const _Position &A, const _Position &B, const _Position &C)
{	
  _Position innerheart, a_inner, b_inner, c_inner, first_cross, second_cross;
  _Position cog_init(0,0,0);
  float ratio = 1;
  float radius = math->inscribedCircleRadius(A,B,C);

  // std::cout<<"inscribedCircleRadius:"<<radius<<std::endl;
  
  innerheart = math->getInnerHeart(A, B, C);
  // std::cout<<"Inner heart:"<<innerheart.x<<","<<innerheart.y<<std::endl;

  if(radius>cogThreshold)
  {
    ratio = (radius-cogThreshold)/radius; 
    
    a_inner = math->formulaLineSection(A, innerheart, ratio);
    b_inner = math->formulaLineSection(B, innerheart, ratio);   
    c_inner = math->formulaLineSection(C, innerheart, ratio);
    
    first_cross = math->getCrossPoint(a_inner,b_inner,cog_init,innerheart);
    second_cross = math->getCrossPoint(a_inner,c_inner,cog_init,innerheart);
    
    // std::cout<<"Ratio:"<<ratio<<std::endl;
    // std::cout<<"A:"<<A.x<<","<<A.y<<" B:"<<B.x<<","<<B.y<<" C:"<<C.x<<","<<C.y<<std::endl;
    // std::cout<<"a:"<<a_inner.x<<","<<a_inner.y<<" b:"<<b_inner.x<<","<<b_inner.y<<" c:"<<c_inner.x<<","<<c_inner.y<<std::endl;

    // std::cout<<"first_cross:"<<first_cross.x<<","<<first_cross.y<<std::endl;
    // std::cout<<"second_cross:"<<second_cross.x<<","<<second_cross.y<<std::endl;

    if(first_cross.x<=max(a_inner.x, b_inner.x) && first_cross.x>=min(a_inner.x, b_inner.x))
    {     
      // std::cout<<"first_cross true"<<std::endl;
      return first_cross;
    }
    if(second_cross.x<=max(a_inner.x, c_inner.x) && second_cross.x>=min(a_inner.x, c_inner.x))
    {
      // std::cout<<"second_cross true"<<std::endl;
      return second_cross;
    }   
  }
  else
  {
    return innerheart;    
  }
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: calculating CoG position and CoG adjust vector,using next foothold,while 1 means to right side
   Input: four feet(end effector) position(x,y,z),Swing leg order and next Swing leg position
   Output: CoG adjust vector
**************************************************************************/
_Position PositionJointGroupController::get_CoG_adj_vec(const _Position &Next_Foothold, unsigned int Swing_Order)
{
	_Position crosspoint;
	switch (Swing_Order)
	{
		case LF:		
		case LB:
		crosspoint = math->getCrossPoint(Pos_ptr->lf+gesture->getSingleShoulderPos(LF), Pos_ptr->rb+gesture->getSingleShoulderPos(RB),Pos_ptr->rf+gesture->getSingleShoulderPos(RF), Pos_ptr->lb+gesture->getSingleShoulderPos(LB));
		return innerTriangle(crosspoint, Pos_ptr->rf+gesture->getSingleShoulderPos(RF), Pos_ptr->rb+gesture->getSingleShoulderPos(RB));		
		case RF:
		case RB:
		crosspoint = math->getCrossPoint(Pos_ptr->rf+gesture->getSingleShoulderPos(RF), Pos_ptr->lb+gesture->getSingleShoulderPos(LB),
		Pos_ptr->lf+gesture->getSingleShoulderPos(LF), Pos_ptr->rb+gesture->getSingleShoulderPos(RB));
		return innerTriangle(crosspoint, Pos_ptr->lf+gesture->getSingleShoulderPos(LF), Pos_ptr->lb+gesture->getSingleShoulderPos(LB));		
	}
}


float PositionJointGroupController::get_adj_pos(float Adj, int t, int T)
{
	float pos = 6 * Adj * pow(t,5) / pow(T,5) - 15 * Adj * pow(t,4) / pow(T,4) + 10 * Adj * pow(t,3) / pow(T,3);
	return pos;
}

void PositionJointGroupController::assign_next_foot()
{
	switch(Leg_Order)
	{
		case LF:
		Pos_start = Pos_ptr->lf;
		Desired_Foot_Pos.assign(Foot_Steps, 6, -Stance_Height);	
		break;
		case RF:
		Pos_start = Pos_ptr->rf;
		Desired_Foot_Pos.assign(Foot_Steps, -6, -Stance_Height);	
		break;
		case LB:
		Pos_start = Pos_ptr->lb;
		Desired_Foot_Pos.assign(Foot_Steps, 6, -Stance_Height);	
		break;
		case RB:
		Pos_start = Pos_ptr->rb;
		Desired_Foot_Pos.assign(Foot_Steps, -6, -Stance_Height);	
		break;
		default:
		break;
	}	
}

void PositionJointGroupController::cog_adj()
{
	_Position adj = {0,0,0};
	// std::cout<<"LOOP COUNT:"<<Loop_Count<<std::endl;
	
	if(Loop_Count<=1)
	{
		Cog_adj = get_CoG_adj_vec(Desired_Foot_Pos,Leg_Order); 
		std::cout<<"COg adjust vector:"<<Cog_adj.x<<", "<<Cog_adj.y<<std::endl;
		if(Leg_Order<2)
		{
			Stance_Num = 5;
		}
		else
		{
			Stance_Num = 40;
		}
		// Stance_Num = round(10 * max(fabs(Cog_adj.x),fabs(Cog_adj.y)));
		// std::cout<<std::endl;
		// std::cout<<"COG:Leg order:"<<Leg_Order<<", "<<Stance_Num<<std::endl;
		// std::cout<<std::endl;
	}

	adj = get_stance_velocity(Cog_adj, Loop_Count);
	cog_pos_assign(adj);
	reverse_kinematics();
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after knowing the exact distance to move CoG, design proper trajectory
   Input: CoG adjust Vector and Loop(1~)
   Output: realtime position
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
   X-axis:
   Pos(t):6 * XD * t^5 / Stance_Num^5 - 15 * XD * t^4 / Stance_Num^4 + 10 * XD * t^3 / Stance_Num^3
   Y-axis:
   Pos(t):6 * YD * t^5 / Stance_Num^5 - 15 * YD * t^4 / Stance_Num^4 + 10 * YD * t^3 / Stance_Num^3
   Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: initialize body pose at first time
   Input: position and Angle_ptr pointer
   Output: none
**************************************************************************/
void PositionJointGroupController::swing_control()
{
	_Position s = {0,0,0};
	// std::cout<<"IMU :"<<round(Imu.pitch*1000)/1000<<", "<<round(Imu.yaw *1000)/1000<<", "<<round(Imu.roll*1000)/1000<<std::endl;

	if(Loop_Count<=Swing_Num)
	{					
		if(Loop_Count>Swing_Num/3*2)
		{
			foot_contact->tdSingleEventConditionDetect(Leg_Order);	
		}
		if(!foot_contact->singleFootContactStatus(Leg_Order))
		{
			s = swing->get_rect_pos(Pos_start, Desired_Foot_Pos, Loop_Count, Swing_Num, Swing_Height);
			// std::cout<<"X diff:"<<Desired_Foot_Pos.x - Pos_start.x<<" Y diff:"<<Desired_Foot_Pos.y - Pos_start.y<<std::endl;

			if(Leg_Order==LF)
				Pos_ptr->lf = Pos_start + s;
			if(Leg_Order==RF)
				Pos_ptr->rf = Pos_start + s;
			if(Leg_Order==LB)
				Pos_ptr->lb = Pos_start + s;
			if(Leg_Order==RB)
				Pos_ptr->rb = Pos_start + s;
		}
	}
	else
	{			
		foot_contact->tdSingleEventConditionDetect(Leg_Order);		
		if(!foot_contact->singleFootContactStatus(Leg_Order))
		{
			Height_recovery_count++;
			on_ground_control(Leg_Order);
			//std::cout<<"ON ground_control is working"<<std::endl;
		}			
	}	
	reverse_kinematics();
}


void PositionJointGroupController::cog_pos_assign(_Position Adj)
{
	Pos_ptr->lf = Pos_ptr->lf - Adj;
	Pos_ptr->rf = Pos_ptr->rf - Adj;
	Pos_ptr->lb = Pos_ptr->lb - Adj;
	Pos_ptr->rb = Pos_ptr->rb - Adj;
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after design the exact trajectory, providing velocity while move the body
   Input: CoG adjust Vector and Loop(1~)
   Output: realtime velocity
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
   X-axis:
   Vel(t):30 * XD * t^4 / Stance_Num^5 - 60 * XD * t^3 / Stance_Num^4 + 30 * XD * t^2 / Stance_Num^3
   Y-axis:
   Vel(t):30 * YD * t^4 / Stance_Num^5 - 60 * YD * t^3 / Stance_Num^4 + 30 * YD * t^2 / Stance_Num^3
   Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/
_Position PositionJointGroupController::get_stance_velocity(_Position Adj_vec, unsigned int Loop)
{
	if(Loop>Stance_Num)
	{
		ROS_ERROR("Time Order wrong while stance");
	}

	_Position stance_vel = {0,0,0};
	stance_vel.x = 30 * Adj_vec.x * pow(Loop,4) / pow(Stance_Num,5) - 60 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,4)
	+ 30 * Adj_vec.x * pow(Loop,2) / pow(Stance_Num,3);
	stance_vel.y = 30 * Adj_vec.y * pow(Loop,4) / pow(Stance_Num,5) - 60 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,4)
	+ 30 * Adj_vec.y * pow(Loop,2) / pow(Stance_Num,3);
	return stance_vel;
}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.24
   Description: after design the exact velocity, providing accelation while move the body
   Input: CoG adjust Vector and Loop(1~)
   Output: realtime accelation
   Formula：(might as well think t1=0 and t2=Stance_Num to reduce calculation)
   X-axis:
   Acc(t):120 * XD * t^3 / Stance_Num^5 - 180 * XD * t^2 / Stance_Num^4 + 60 * XD * t / Stance_Num^3
   Y-axis:
   Acc(t):120 * YD * t^3 / Stance_Num^5 - 180 * YD * t^2 / Stance_Num^4 + 60 * YD * t / Stance_Num^3
   Meantime,XD,YD is the distance to the desired position,Stance_Num is the stance time for CoG adjusting
**************************************************************************/
_Position PositionJointGroupController::get_stance_acceration(_Position Adj_vec, unsigned int Loop)
{
	if(Loop>Stance_Num)
	{
		ROS_ERROR("Time Order wrong while stance");
	}

	_Position stance_acc = {0,0,0};
	stance_acc.x = 120 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,5) - 180 * Adj_vec.x * pow(Loop,2) / pow(Stance_Num,4)
	+ 60 * Adj_vec.x * Loop / pow(Stance_Num,3);
	stance_acc.y = 120 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,5) - 180 * Adj_vec.y * pow(Loop,2) / pow(Stance_Num,4)
	+ 60 * Adj_vec.y * Loop / pow(Stance_Num,3);
	return stance_acc;
}

std::vector<double> PositionJointGroupController::vec_assign(Angle_Ptr Angle)
{
	std::vector<double> vec;
	vec.clear();
	vec.push_back(Angle->lb.hip);
	vec.push_back(Angle->lb.knee);
	vec.push_back(Angle->lb.pitch);

	vec.push_back(Angle->lf.hip);
	vec.push_back(Angle->lf.knee);
	vec.push_back(Angle->lf.pitch);

	vec.push_back(Angle->rb.hip);
	vec.push_back(Angle->rb.knee);
	vec.push_back(Angle->rb.pitch);

	vec.push_back(Angle->rf.hip);
	vec.push_back(Angle->rf.knee);
	vec.push_back(Angle->rf.pitch);

	return vec;
}


}
// PLUGINLIB_DECLARE_CLASS(qr_control, PositionJointGroupController,
//                         qr_control::PositionJointGroupController,
//                         controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(qr_control::PositionJointGroupController,controller_interface::ControllerBase)
