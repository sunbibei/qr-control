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
	void PositionJointGroupController::sensor_height()
	{
		Angle_ptr->lf.knee = *(jnt_poss_[0]);
		Angle_ptr->lf.hip  = *(jnt_poss_[1]);		
		Angle_ptr->lf.pitch= *(jnt_poss_[2]);

    Angle_ptr->rf.knee = *(jnt_poss_[3]);
		Angle_ptr->rf.hip  = *(jnt_poss_[4]);		
		Angle_ptr->rf.pitch= *(jnt_poss_[5]);

    Angle_ptr->lb.knee = *(jnt_poss_[6]);
		Angle_ptr->lb.hip  = *(jnt_poss_[7]);		
		Angle_ptr->lb.pitch= *(jnt_poss_[8]);
		
    Angle_ptr->rb.knee = *(jnt_poss_[9]);
		Angle_ptr->rb.hip  = *(jnt_poss_[10]);		
		Angle_ptr->rb.pitch= *(jnt_poss_[11]);	

     
  // p = math->cal_formula(Angle_ptr->lf),  real_h[LF] = p.z;
  // p = math->cal_formula(Angle_ptr->rf),  real_h[RF] = p.z;
  // p = math->cal_formula(Angle_ptr->lb),  real_h[LB] = p.z;
  // p = math->cal_formula(Angle_ptr->rb),  real_h[RB] = p.z;	
	}


/**************************************************************************
   Description: initializition from robot_description
**************************************************************************/
	bool PositionJointGroupController::init(hardware_interface::RobotHW* robot, ros::NodeHandle &n)
	{
		// urdf::Model urdf;
		// if(!urdf.initParam("robot_description"))
		// {
		// 	ROS_ERROR("Failed to parse urdf file");
		// 	return false;
		// }
		// n_joints_ = 0;
		// PositionJointInterface* iface = robot->get<PositionJointInterface>();
		// while(true) 
		// {
		// 	std::string joint_name;
		// 	std::string param_name = std::string("joint_" + std::to_string(n_joints_));
		// 	if (ros::param::get(param_name.c_str(), joint_name)) 
		// 	{
		// 		std::cout << "Get Joint Name: " << joint_name << std::endl;
		// 		joints_.push_back(iface->getHandle(joint_name));
		// 	} 
		// 	else 
		// 	{
		// 		break;
		// 	}
		// 	++n_joints_;
		// }
		auto jnt_manager = middleware::JointManager::instance();
    for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR})
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW})
      joint_handles_.push_back(jnt_manager->getJointHandle(leg, jnt));

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
		foot_contact->setThreshold(900, 960, 720, 1080);
		foot_contact->init();

		swing = new Swing();
		swing->init();

		math = new Math();
		math->init();

		gesture = new Gesture();
		gesture->init(robot);

    Commands tmp(0,0,false);
    
    for(int i = 0;i<Joint_Num;i++)
		{
			commands.push_back(tmp);
		}
		for(int i=0;i<Leg_Num;i++)
		{
			height.push_back(-Stance_Height);
      SupportLeg.push_back(true);
		}

    __initAllofData();

    Isstand_stable = false;
	  All_on_ground = false;
		std::cout<<"System Init Succeed!"<<std::endl;
		
		// std::cout<<"Robot has been initialized! Please input transfer point to continue:";

		// std::cin>>Transfer_Point;

		return true;
	}
void PositionJointGroupController::__initAllofData() {
  jnt_poss_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  jnt_vels_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  jnt_tors_.reserve(LegType::N_LEGS * JntType::N_JNTS);

  auto jnt_manager = middleware::JointManager::instance();
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
      auto j = jnt_manager->getJointHandle(leg, jnt);
      jnt_poss_.push_back(j->joint_position_const_pointer());
      jnt_vels_.push_back(j->joint_velocity_const_pointer());
      jnt_tors_.push_back(j->joint_torque_const_pointer());
    }
  }
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
	command_init();
	// count_loop++;
	// if(count_loop==Transfer_Point)
	// { 
  Switch = 0;
	// 	std::cout<<count_loop<<" Starting backward!"<<std::endl;
	// }
	
	gesture->updateImuData();
  // gesture->printImuAngle();

	foot_contact->footForceDataUpdate(*(td_handles_[0].getForce()), *(td_handles_[1].getForce()), *(td_handles_[2].getForce()), *(td_handles_[3].getForce()));
	// foot_contact->printForce();

	// if(Isstand_stable)//force_control
	// {
	// 	contact_keep();
 //    // foot_contact->printForce();
 //    // std::cout<<"desired Height:"<<height[0]<<" "<<height[1]<<" "<<height[2]<<" "<<height[3]<<" "<<std::endl;
 //    // std::cout<<"Calcued Height:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
	// }


	switch (Time_Order)
	{
	case 0: //init gesture
	pose_init();	
	flow_control(Time_Order);	
	// flow_control_turn(Time_Order);		
	break;

	case 1:		
	// hardware_delay_test();  
  foot_contact->tdEventDetect();
  if(Loop_Count>10)
  {
	  Isstand_stable = stand_stable(foot_contact->contactStatus());
  }
  if(Isstand_stable)
  	gesture->imuCalibration();	
	flow_control(Time_Order);
	// flow_control_turn(Time_Order);
	break;

	case 2:
	gesture->updateShoulderPos();	
	
	cog_adj();
	flow_control(Time_Order);
	// cog_adj_turn();
	// std::cout<<"cog_adj,Leg_Order:"<<Leg_Order<<std::endl;    
	// flow_control_turn(Time_Order);	
	break;
	
	case 3://swing leg	

	swing_control();
	// std::cout<<"swing Leg_Order:"<<Leg_Order<<std::endl;			
	flow_control(Time_Order);		
	break;

  case 4:
  foot_contact->tdEventDetect();
  All_on_ground = contact_keep(foot_contact->contactStatus());		
  flow_control(Time_Order);
  break;

	default:break;
}

Loop_Count++;

command_assign(Angle_ptr);

forward_control(1,0,0);//KP,Kv,Kd
std::cout<<"update: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;

for (size_t i = 0; i < joint_handles_.size(); ++i) 
{
	joint_handles_[i]->updateJointCommand(commands[i].position_);
}
// print_command();
if(joint_state_publisher_ && joint_state_publisher_->trylock())
{
	joint_state_publisher_->msg_.data.clear();
	for(int i=0; i<Joint_Num; i++)
	{
		joint_state_publisher_->msg_.data.push_back(commands[i].position_);
	}
	joint_state_publisher_->unlockAndPublish();
}
}





void PositionJointGroupController::command_init()
{
	for(int i=0;i<Joint_Num;i++)
	{
		commands[i].has_velocity_ = false;
	}
}
void PositionJointGroupController::print_command()
{
  std::cout<<"Joint Command: position ";
  for(int i=0;i<Joint_Num;i++)
	{
		std::cout<<commands[i].position_<<" ";
	}
	std::cout<<std::endl;
	std::cout<<"Joint Command: velocity ";
  for(int i=0;i<Joint_Num;i++)
	{
		std::cout<<commands[i].has_velocity_<<" ";
	}
	std::cout<<std::endl;
	std::cout<<"Joint Command: velocity ";
  for(int i=0;i<Joint_Num;i++)
	{
		std::cout<<commands[i].velocity_<<" ";
	}
	std::cout<<std::endl;
}

bool PositionJointGroupController::stand_stable(std::vector<bool> IsContact)
{ 	
  int phase = 1, t = 100;
  float h = 0, mean = 0, beta = 0.025;  
  
  std::cout<<"stand stable:"<<IsContact[LF]<<" "<<IsContact[RF]<<" "<<IsContact[LB]<<" "<<IsContact[RB]<<std::endl;
  foot_contact->printForce();

  if(IsContact[LF] && IsContact[RF] && IsContact[LB] &&IsContact[RB])
  {
    phase = 2;
  }
//phase one: make sure all legs are in touch with ground, using foot force"
	if(phase == 1)
	{
    std::cout<<"phase one: make sure all legs are in touch with ground, using foot force"<<std::endl;
		for(int i=0;i<Leg_Num;i++)
  	{
  		if(!IsContact[i])
  		{
  			height[i] -= beta;
  		}  		
 	  }	
	}
  //phase two: make sure overall force are balanced
  if(phase == 2)
  {
    std::cout<<"phase two: make sure overall force are balanced"<<std::endl;

    mean = (foot_contact->getForceData(LF) + foot_contact->getForceData(RF)
           +foot_contact->getForceData(LB) + foot_contact->getForceData(RB))/4.0;

    for(int i=0;i<Leg_Num;i++)
    {    
      if(foot_contact->getForceData(i) < mean-t)
      { 
        height[i] -= beta;         
      } 
      else if(foot_contact->getForceData(i) > mean+t)
      { 
        height[i] += beta;         
      } 
      else
      {
        phase++;
      }
    }   

    if(phase == 6)
    {
      phase = 3;
    }
  }
  //phase three: make sure overall height are close to desired height
  if(phase == 3)
  {
    std::cout<<"phase three: make sure overall height are close to desired height"<<std::endl;

  	h = (height[LF]+height[RF]+height[LB]+height[RB])/4.0;
    // std::cout<<"stand stable: h"<<h<<" "<<fabs(Stance_Height - fabs(h))<<std::endl;
  	if(fabs(Stance_Height - fabs(h))>beta)
  	{
  		for(int i=0;i<Leg_Num;i++)
  		{  		
  			height[i] -= Sgn(Stance_Height + h) * beta;  		 		
 	  	}	
 		}
 		else
 		{
      if(IsContact[LF] && IsContact[RF] && IsContact[LB] &&IsContact[RB])
      {
        phase = 4;
      }
 			else
      {
        phase = 2;
      }
 		}
  } 

  // using adjusted height to calculate angle in next loop
  Pos_ptr->lf.z = height[LF];
  Pos_ptr->rf.z = height[RF];
  Pos_ptr->lb.z = height[LB];
  Pos_ptr->rb.z = height[RB]; 
  reverse_kinematics();
  // std::cout<<"stable stand: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
 
  //phase three: record IMU data, and always keep this angle since then
  if(phase == 4)
  {  	
    foot_contact->setConst(foot_contact->getForceData(LF), foot_contact->getForceData(RF),
           foot_contact->getForceData(LB), foot_contact->getForceData(RB));
    foot_contact->printConst();
  	return true;
  }
  return false;
}
//four legs are all on ground, then under force limit recover height
bool PositionJointGroupController::contact_keep(std::vector<bool> IsContact)
{
  int phase = 1, t = 100;
  float h = 0, d_h = 0, mean = 0, beta = 0.025;  
  std::vector<float> h_adj(Leg_Num,0);
  std::cout<<"contact_keep:"<<IsContact[LF]<<" "<<IsContact[RF]<<" "<<IsContact[LB]<<" "<<IsContact[RB]<<std::endl;
  foot_contact->printForce();

  if(IsContact[LF] && IsContact[RF] && IsContact[LB] &&IsContact[RB])
  {
    phase = 2;
  }
//phase one: make sure all legs are in touch with ground, using foot force"
  if(phase == 1)
  {
    std::cout<<"phase one: make sure all legs are in touch with ground, using foot force"<<std::endl;
    for(int i=0;i<Leg_Num;i++)
    {
      if(!IsContact[i])
      {
        h_adj[i] -= beta;
      }     
    } 
  }
  //phase two: make sure overall force are balanced
  if(phase == 2)
  {
    std::cout<<"phase two: make sure overall force are balanced"<<std::endl;

    mean = (foot_contact->getForceData(LF) + foot_contact->getForceData(RF)
           +foot_contact->getForceData(LB) + foot_contact->getForceData(RB))/4.0;

    for(int i=0;i<Leg_Num;i++)
    {    
      if(foot_contact->getForceData(i) < mean-t)
      { 
        h_adj[i] -= beta;         
      } 
      else if(foot_contact->getForceData(i) > mean+t)
      { 
        h_adj[i] += beta;         
      } 
      else
      {
        phase++;
      }
    }   

    if(phase == 6)
    {
      phase = 3;
    }
  }
  //phase three: make sure overall height are close to desired height
  if(phase == 3)
  {
    std::cout<<"phase three: make sure overall height are close to desired height"<<std::endl;

    h = (Pos_ptr->lf.z + Pos_ptr->rf.z + Pos_ptr->lb.z + Pos_ptr->rb.z)/4.0;
    d_h = (height[LF] + height[RF] + height[LB] + height[RB])/4.0;
    // std::cout<<"stand stable: h"<<h<<" "<<fabs(Stance_Height - fabs(h))<<std::endl;
    if(fabs(h - d_h)>beta)
    {
      for(int i=0;i<Leg_Num;i++)
      {     
        h_adj[i] += Sgn(d_h - h) * beta;         
      } 
    }
    else
    {
      phase = 4;
    }
  } 

  // using adjusted height to calculate angle in next loop
  Pos_ptr->lf.z += h_adj[LF];
  Pos_ptr->rf.z += h_adj[RF];
  Pos_ptr->lb.z += h_adj[LB];
  Pos_ptr->rb.z += h_adj[RB]; 
  reverse_kinematics();
  // std::cout<<"stable stand: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
 
  //phase three: record IMU data, and always keep this angle since then
  if(phase == 4)
  {       
    return true;
  }
  return false;
  // std::cout<<"Support leg:"<<SupportLeg[LF]<<" "<<SupportLeg[RF]<<" "<<SupportLeg[LB]<<" "<<SupportLeg[RB]<<std::endl;
  // std::cout<<"Contact status:"<<IsContact[LF]<<" "<<IsContact[RF]<<" "<<IsContact[LB]<<" "<<IsContact[RB]<<std::endl;
  // foot_contact->printForce();

  // float beta = 0.1, count = 0, h = 0;

  // h = (height[LF]+height[RF]+height[LB]+height[RB])/4.0;
  // h = h -(Pos_ptr->lf.z+Pos_ptr->rf.z+Pos_ptr->lb.z+Pos_ptr->rb.z)/4.0;

  // Pos_ptr->lf.z += h;
  // Pos_ptr->rf.z += h;
  // Pos_ptr->lb.z += h;
  // Pos_ptr->rb.z += h;

  // if(SupportLeg[LF])
  // { 
  //   if(IsContact[LF]==false)
  //   {
  //     Pos_ptr->lf.z -= beta;      
  //   }
  //   else if(fabs(Pos_ptr->lf.z-height[LF]) > beta)
  //   {
  //     Pos_ptr->lf.z -= Sgn(Pos_ptr->lf.z-height[LF]) * beta * 0.5;      
  //   }
  //   else
  //   {
  //     count++;
  //   }
  // }
  // if(SupportLeg[RF])
  // { 
  //   if(IsContact[RF]==false)
  //   {
  //     Pos_ptr->rf.z -= beta;      
  //   }
  //   else if(fabs(Pos_ptr->rf.z-height[RF]) > beta)
  //   {
  //     Pos_ptr->rf.z -= Sgn(Pos_ptr->rf.z-height[RF]) * beta * 0.5;      
  //   }
  //   else
  //   {
  //     count++;
  //   }
  // }
  // if(SupportLeg[LB])
  // { 
  //   if(IsContact[LB]==false)
  //   {
  //     Pos_ptr->lb.z -= beta;      
  //   }
  //   else if(fabs(Pos_ptr->lb.z-height[LB]) > beta)
  //   {
  //     Pos_ptr->lb.z -= Sgn(Pos_ptr->lb.z-height[LB]) * beta * 0.5;
  //   }
  //   else
  //   {
  //     count++;
  //   }
  // }
  // if(SupportLeg[RB])
  // { 
  //   if(IsContact[RB]==false)
  //   {
  //     Pos_ptr->rb.z -= beta;      
  //   }
  //   else if(fabs(Pos_ptr->rb.z-height[RB]) > beta)
  //   {
  //     Pos_ptr->rb.z -= Sgn(Pos_ptr->rb.z-height[RB]) * beta * 0.5;
  //   }
  //   else
  //   {
  //     count++;
  //   }
  // }
  
  // reverse_kinematics(); 
  // std::cout<<"contact: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
  
  // return (count==4) ? true:false;
  // float beta_up = 0.0001, beta_down = 0.0001, t_up = 3800, t_down = 1000;
  // if(SupportLeg[LF])
  // {    
  //   if(foot_contact->getForceData(LF) < t_down)
  //   {
  //     Pos_ptr->lf.z -= beta_down * (t_down - foot_contact->getForceData(LF));
  //   }
  //   else if(foot_contact->getForceData(LF) > t_up)
  //   {      
  //     Pos_ptr->lf.z += beta_up * (foot_contact->getForceData(LF) - t_up);
  //   }
  // }

  // if(SupportLeg[RF])
  // {
  //   if(foot_contact->getForceData(RF) < t_down)
  //   {
  //     Pos_ptr->rf.z -= beta_down * (t_down - foot_contact->getForceData(RF));
  //   }
  //   else if(foot_contact->getForceData(RF) > t_up)
  //   {
  //     Pos_ptr->rf.z += beta_up * (foot_contact->getForceData(RF) - t_up);
  //   }
  // }
  // if(SupportLeg[LB])
  // {
  //   if(foot_contact->getForceData(LB) < t_down)
  //   {
  //     Pos_ptr->lb.z -= beta_down * (t_down - foot_contact->getForceData(LB));
  //   }
  //   else if(foot_contact->getForceData(LB) > t_up)
  //   {
  //     Pos_ptr->lb.z += beta_up * (foot_contact->getForceData(LB) - t_up);
  //   }
  // }
  // if(SupportLeg[RB])
  // {
  //   if(foot_contact->getForceData(RB) < t_down)
  //   {
  //     Pos_ptr->rb.z -= beta_down * (t_down - foot_contact->getForceData(RB));
  //   }
  //   else if(foot_contact->getForceData(RB) > t_up)
  //   {
  //     Pos_ptr->rb.z += beta_up * ( foot_contact->getForceData(RB) - t_up);
  //   }
  // }

  // reverse_kinematics(); 
  // std::cout<<"contact_keep: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
  
  //step one: calculate real height using joint coder value in forward kinematics
  // _Position p;
  // std::vector<float> real_h(Leg_Num,0);
  // update_angle();
  // p = math->cal_formula(Angle_ptr->lf),  real_h[LF] = p.z;
  // p = math->cal_formula(Angle_ptr->rf),  real_h[RF] = p.z;
  // p = math->cal_formula(Angle_ptr->lb),  real_h[LB] = p.z;
  // p = math->cal_formula(Angle_ptr->rb),  real_h[RB] = p.z;
  //step two:adjust error in height
  
  // Pos_ptr->lf.z  -= height[LF]-real_h[LF];
  // Pos_ptr->rf.z  -= height[RF]-real_h[RF];
  // Pos_ptr->lb.z  -= height[LB]-real_h[LB];
  // Pos_ptr->rb.z  -= height[RB]-real_h[RB];
}

void PositionJointGroupController::forward_control(float Kp,float Kv,float Kd)
{
	float result = 0; 

	for(int i=0;i<Joint_Num;i++)
	{
		if(commands[i].has_velocity_)
		{
			commands[i].position_ = single_forward_control(
				commands[i].position_, commands[i].velocity_, Kp, Kv ,Kd,
				*(jnt_poss_[i]), *(jnt_vels_[i]));
		}
	}	
}

float PositionJointGroupController::single_forward_control(
	float d_pos, float d_vel, float Kp, float Kv, float Kd, float f_pos, float f_vel)
{
  float err_pos = d_pos - f_pos;
  float result = Kp*err_pos + Kd*d_vel - Kv*f_vel;
  // return result;
  return d_pos;
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


void PositionJointGroupController::state_transfer()
{
	std::cout<<" State transfering!"<<std::endl;
	if(Time_Order==2 | Time_Order==4)
	{
		Time_Order = 4;
	}		
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
			std::cin>>Time_Order;	
			// Time_Order=1;
		}
		break;		

		case 1:
		// if(Loop_Count>=30)
    if(Isstand_stable) 
		{
			foot_contact->clear();		
			Loop_Count = 0;
			// std::cout<<"Imu data calibration done!"<<std::endl;
			// Time_Order=2;
      std::cout<<"Robot has stand stable! Please input 2 to continue:"; 
      std::cin>>Time_Order; 
		}
		break;	

		case 2:
		if(Loop_Count>=Stance_Num)
		{
			Loop_Count = 0;			
			// std::cout<<"flow_control: cog done"<<std::endl;
			Time_Order=3;	      
      // std::cout<<"Robot moving CoG! Please input 3 to continue:";
      // std::cin>>Time_Order; 
			assign_next_foot();
			std::cout<<"*******-----------------------------------case 3---------------------------------------*******"<<std::endl;

		}							
		break;	

		case 3:
		if(foot_contact->isLegOnGround())
		// if(Loop_Count>=Swing_Num)
		{
			// std::cout<<"flow_control: swing done"<<std::endl;
			test_tmp = 0;
			Loop_Count = 0;
      for(int i=0;i<Leg_Num;i++)
      {
        SupportLeg[Leg_Order] = true;
      }
			foot_contact->clear();	

			Time_Order = 4;	      
      // std::cout<<"Robot swing done! Please input 4 to continue:";
      // std::cin>>Time_Order; 			
			Leg_Order = (Leg_Order>1)?Leg_Order-2:3-Leg_Order;		
			std::cout<<"*******-----------------------------------case 4----------------------------------------*******"<<std::endl;
		}					
		break;

    case 4:
    if(All_on_ground)
    {
      All_on_ground = false;
      Loop_Count = 0;
      foot_contact->clear(); 
      // std::cout<<"Robot Contact adjust done! Please input 2 to continue:";
      // std::cin>>Time_Order;    
      Time_Order = 2;
      // std::cout<<"All_on: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
      std::cout<<"*******------------------------------------case 2---------------------------------------*******"<<std::endl;

    }
		
		default:break;
	}
}

/**************************************************************************
   Description: Ensure the foot is on ground by recursive decreasing height
   Input: lf,rf,lb,rb corresponding to 0,1,2,3
**************************************************************************/
void PositionJointGroupController::on_ground_control(int legId)
{
	double err = 0.3;  
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
      swing_adj_CoG = (b_inner.x>c_inner.x) ? b_inner - first_cross : c_inner - first_cross;
      swing_adj_CoG.x = swing_adj_CoG.x/2.0;
      swing_adj_CoG.y = swing_adj_CoG.y/2.0;
      // std::cout<<"swing_adj_CoG: "<<swing_adj_CoG.x <<" "<<swing_adj_CoG.y<<std::endl;
      return first_cross;
    }
    if(second_cross.x<=max(a_inner.x, c_inner.x) && second_cross.x>=min(a_inner.x, c_inner.x))
    {
      // std::cout<<"second_cross true"<<std::endl;
      swing_adj_CoG = (b_inner.x>c_inner.x) ? b_inner - second_cross : c_inner - second_cross;
      swing_adj_CoG.x = swing_adj_CoG.x/2.0;
      swing_adj_CoG.y = swing_adj_CoG.y/2.0;
      // std::cout<<"swing_adj_CoG: "<<swing_adj_CoG.x <<" "<<swing_adj_CoG.y<<std::endl;
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

void PositionJointGroupController::assign_next_foot()
{
	switch(Leg_Order)
	{
		case LF:
		Pos_start = Pos_ptr->lf;
		Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[LF]);	
		break;
		case RF:
		Pos_start = Pos_ptr->rf;
		Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[RF]);	
		break;
		case LB:
		Pos_start = Pos_ptr->lb;
		Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[LB]);	
		break;
		case RB:
		Pos_start = Pos_ptr->rb;
		Desired_Foot_Pos.assign(Foot_Steps, Pos_start.y, height[RB]);	
		break;
		default:
		break;
	}	
}

void PositionJointGroupController::cog_adj()
{
	// Switch = 1;
	_Position adj = {0,0,0};
  std::vector<float> h_adj(Leg_Num,0);
  std::vector<float> h_adj_step(Leg_Num,0);
	
	if(Loop_Count<=1)
	{
		Cog_adj = get_CoG_adj_vec(Desired_Foot_Pos, Leg_Order); 
    if(SupportLeg[LF])
    {        
      h_adj[LF] = -(height[LF] - Pos_ptr->lf.z);  
    }
    if(SupportLeg[RF])
    {   
      h_adj[RF] = -(height[RF] - Pos_ptr->rf.z);
    }  
    if(SupportLeg[LB])
    {   
      h_adj[LB] = -(height[LB] - Pos_ptr->lb.z);
    }  
    if(SupportLeg[RB])
    {    
      h_adj[RB] = -(height[RB] - Pos_ptr->rb.z);
    }      
		// std::cout<<"CoG adjust vector:"<<Cog_adj.x<<", "<<Cog_adj.y<<std::endl;		
		Stance_Num = (Leg_Order<2) ? 1:50;		
	}

	adj = get_stance_velocity(Cog_adj, Loop_Count);
  // std::cout<<"cog_adj h_adj_step:";
  for(int i=0;i<h_adj.size();i++)
  {
    if(h_adj[i]!=0)
    {
      // h_adj_step[i] = (fabs(h_adj[i])>0.1) ? Sgn(h_adj[i]) * 0.05 : h_adj[i];
      h_adj_step[i] = 0;
      // std::cout<<h_adj_step[i]<<" ";
    }
  }
  std::cout<<std::endl;
  

	cog_pos_assign(adj,h_adj_step);
	reverse_kinematics();
  // std::cout<<"cog_adj after: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;

}

void PositionJointGroupController::swing_control()
{
	// Switch = 1;
	EV3 foot_vel(0,0,0),joint_vel(0,0,0),joint_pos(0,0,0);
	_Position s = {0,0,0};
	_Angle_Leg tmp;
	// std::cout<<"IMU :"<<round(Imu.pitch*1000)/1000<<", "<<round(Imu.yaw *1000)/1000<<", "<<round(Imu.roll*1000)/1000<<std::endl;

	if(Loop_Count<=Swing_Num)
	{					
		if(Loop_Count>Swing_Num/3*2)
		{
			foot_contact->tdSingleEventConditionDetect(Leg_Order);	
		}
		if(!foot_contact->singleFootContactStatus(Leg_Order))
		{
			s = swing->compoundCycloidPosition(Pos_start, Desired_Foot_Pos, Loop_Count, Swing_Num, Swing_Height);
			foot_vel = swing->compoundCycloidVelocity(Pos_start, Desired_Foot_Pos, Loop_Count, Swing_Num, Swing_Height);
			// std::cout<<"X diff:"<<Desired_Foot_Pos.x - Pos_start.x<<" Y diff:"<<Desired_Foot_Pos.y - Pos_start.y<<std::endl;
/*      std::cout<<"foot_velocity: ";
  		for(int i=0;i<3;i++)
			{
				std::cout<<foot_vel(i)<<" ";
			}
			std::cout<<std::endl;*/

			if(Leg_Order==LF)
      {
        SupportLeg[LF] = false;
				Pos_ptr->lf = Pos_start + s;
      }
			if(Leg_Order==RF)
      {
        SupportLeg[RF] = false;
				Pos_ptr->rf = Pos_start + s;
      }
			if(Leg_Order==LB)
      {
        SupportLeg[LB] = false;
				Pos_ptr->lb = Pos_start + s;
      }
			if(Leg_Order==RB)
      {
        SupportLeg[RB] = false;
				Pos_ptr->rb = Pos_start + s;
      }
		}
		//cog moving
		if(Loop_Count<=Swing_Num/3*2)
		{
		  Stance_Num = Swing_Num/3*2;
		  s = get_stance_velocity(swing_adj_CoG, Loop_Count);
		  cog_swing_assign(s, Leg_Order);
		}
    //vel cal
		if(Leg_Order==LF)
	  {			
			joint_pos(0) = Angle_ptr->lf.pitch;
			joint_pos(1) = Angle_ptr->lf.hip;
			joint_pos(2) = Angle_ptr->lf.knee;
			if(math->isJacobInvertible(joint_pos))
			{
				commands[0].has_velocity_ = true;
			  commands[1].has_velocity_ = true;
			  commands[2].has_velocity_ = true;
			}		 
			joint_vel = math->footVelToJoint(joint_pos,foot_vel);
			commands[0].velocity_ = joint_vel(2);
			commands[1].velocity_ = joint_vel(1);
			commands[2].velocity_ = joint_vel(0);
		}
    if(Leg_Order==RF)
	  {			
			joint_pos(0) = Angle_ptr->rf.pitch;
			joint_pos(1) = Angle_ptr->rf.hip;
			joint_pos(2) = Angle_ptr->rf.knee;
			if(math->isJacobInvertible(joint_pos))
			{
				commands[3].has_velocity_ = true;
			  commands[4].has_velocity_ = true;
			  commands[5].has_velocity_ = true;
			}		 
			joint_vel = math->footVelToJoint(joint_pos,foot_vel);
			commands[3].velocity_ = joint_vel(2);
			commands[4].velocity_ = joint_vel(1);
			commands[5].velocity_ = joint_vel(0);
		}
		if(Leg_Order==LB)
	  {					
			joint_pos(0) = Angle_ptr->lb.pitch;
			joint_pos(1) = Angle_ptr->lb.hip;
			joint_pos(2) = Angle_ptr->lb.knee;
			if(math->isJacobInvertible(joint_pos))
			{
				commands[6].has_velocity_ = true;
			  commands[7].has_velocity_ = true;
			  commands[8].has_velocity_ = true;
			}		 
			joint_vel = math->footVelToJoint(joint_pos,foot_vel);
			commands[6].velocity_ = joint_vel(2);
			commands[7].velocity_ = joint_vel(1);
			commands[8].velocity_ = joint_vel(0);

			// tmp = math->cal_kinematics(Pos_start, 1);
  	// 	std::cout<<"befor: "<<tmp.hip<<"  ";
  	// 	tmp = math->cal_kinematics(Desired_Foot_Pos, 1);
  	// 	std::cout<<"after: "<<tmp.hip<<std::endl;
			// test_tmp += joint_vel(1);
			// std::cout<<"jifen: "<<test_tmp<<std::endl;
		}	
		
		if(Leg_Order==RB)
	  {			
			joint_pos(0) = Angle_ptr->rb.pitch;
			joint_pos(1) = Angle_ptr->rb.hip;
			joint_pos(2) = Angle_ptr->rb.knee;
			if(math->isJacobInvertible(joint_pos))
			{
				commands[9].has_velocity_ = true;
			  commands[10].has_velocity_ = true;
			  commands[11].has_velocity_ = true;
			}		 
			joint_vel = math->footVelToJoint(joint_pos,foot_vel);
			commands[9].velocity_ = joint_vel(2);
			commands[10].velocity_ = joint_vel(1);
			commands[11].velocity_ = joint_vel(0);
		}
		
	
	}
	else
	{			
		foot_contact->tdSingleEventConditionDetect(Leg_Order);		
		if(!foot_contact->singleFootContactStatus(Leg_Order))
		{
      SupportLeg[Leg_Order] = false;
			Height_recovery_count++;
			on_ground_control(Leg_Order);
			//std::cout<<"ON ground_control is working"<<std::endl;
		}			
	}	
	reverse_kinematics();
}

void PositionJointGroupController::cog_adj_backward()
{
	_Position adj = {0,0,0};	
	if(Loop_Count<=1)
	{
		Cog_adj = get_CoG_adj_vec(Desired_Foot_Pos,Leg_Order); 
		// std::cout<<"CoG adjust vector:"<<Cog_adj.x<<", "<<Cog_adj.y<<std::endl;	
		if(Leg_Order<2)
		{
			Stance_Num = 50;		
		}
		else
		{
			Stance_Num = 1;
			// Stance_Num = 20;
		}			
	}
	adj = get_stance_velocity(Cog_adj, Loop_Count);
	// cog_pos_assign(adj);
	reverse_kinematics();
}

void PositionJointGroupController::flow_control_backward(int timeorder)
{
	switch(timeorder)
	{
		case 2:
		if(Loop_Count>=Stance_Num)
		{
			Loop_Count = 0;			
			// std::cout<<"flow_control_backward: cog done"<<std::endl;
			Time_Order=4;				
			assign_next_foot_backward();
		}							
		break;			

		case 4:
		// if(foot_contact->isLegOnGround())
		if(Loop_Count>=Swing_Num)
		{
			Loop_Count = 0;
			foot_contact->clear();		
			Time_Order = 2;					
			Leg_Order = (Leg_Order>1) ? 3-Leg_Order:Leg_Order+2;									
			// std::cout<<"flow_control_backward: swing done"<<std::endl;
		}					
		break;		
		default:break;
	}
}

void PositionJointGroupController::assign_next_foot_backward()
{
	switch(Leg_Order)
	{
		case LF:
		Pos_start = Pos_ptr->lf;
		Desired_Foot_Pos.assign(-Foot_Steps, 6, -Stance_Height);	
		break;
		case RF:
		Pos_start = Pos_ptr->rf;
		Desired_Foot_Pos.assign(-Foot_Steps, -6, -Stance_Height);	
		break;
		case LB:
		Pos_start = Pos_ptr->lb;
		Desired_Foot_Pos.assign(-Foot_Steps, 6, -Stance_Height);	
		break;
		case RB:
		Pos_start = Pos_ptr->rb;
		Desired_Foot_Pos.assign(-Foot_Steps, -6, -Stance_Height);	
		break;
		default:
		break;
	}	
}

void PositionJointGroupController::cog_pos_assign(_Position Adj, std::vector<float> height_adj)
{
	Pos_ptr->lf = Pos_ptr->lf - Adj;
	Pos_ptr->rf = Pos_ptr->rf - Adj;
	Pos_ptr->lb = Pos_ptr->lb - Adj;
	Pos_ptr->rb = Pos_ptr->rb - Adj;

  Pos_ptr->lf.z -= height_adj[LF];
  Pos_ptr->rf.z -= height_adj[RF];
  Pos_ptr->lb.z -= height_adj[LB];
  Pos_ptr->rb.z -= height_adj[RB];
}

void PositionJointGroupController::cog_swing_assign(_Position Adj, int legId)
{	
	switch(Leg_Order)
	{
		case LF:
			Pos_ptr->rf = Pos_ptr->rf - Adj;
			Pos_ptr->lb = Pos_ptr->lb - Adj;
			Pos_ptr->rb = Pos_ptr->rb - Adj;
		break;
		case RF:
			Pos_ptr->lf = Pos_ptr->lf - Adj;
			Pos_ptr->lb = Pos_ptr->lb - Adj;
			Pos_ptr->rb = Pos_ptr->rb - Adj;
		break;
		case LB:
			Pos_ptr->lf = Pos_ptr->lf - Adj;
			Pos_ptr->rf = Pos_ptr->rf - Adj;
			Pos_ptr->rb = Pos_ptr->rb - Adj;
		break;
		case RB:
			Pos_ptr->lf = Pos_ptr->lf - Adj;
			Pos_ptr->rf = Pos_ptr->rf - Adj;
			Pos_ptr->lb = Pos_ptr->lb - Adj;
		break;
		default:	break;
	}	
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
		std::cout<<"Loop:"<<Loop<<" Stance_Num:"<<Stance_Num<<std::endl;
		ROS_ERROR("Time Order wrong while stance");
	}

	_Position stance_vel = {0,0,0};
	stance_vel.x = 30 * Adj_vec.x * pow(Loop,4) / pow(Stance_Num,5) - 60 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,4)
	+ 30 * Adj_vec.x * pow(Loop,2) / pow(Stance_Num,3);
	stance_vel.y = 30 * Adj_vec.y * pow(Loop,4) / pow(Stance_Num,5) - 60 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,4)
	+ 30 * Adj_vec.y * pow(Loop,2) / pow(Stance_Num,3);
	return stance_vel;
}

float PositionJointGroupController::get_stance_velocity(float adj, unsigned int Loop)
{
  if(Loop>Stance_Num)
  {
    std::cout<<"Loop:"<<Loop<<" Stance_Num:"<<Stance_Num<<std::endl;
    ROS_ERROR("Time Order wrong while stance");
  }

  float stance_vel = 0;
  stance_vel = 30 * adj * pow(Loop,4) / pow(Stance_Num,5) - 60 * adj * pow(Loop,3) / pow(Stance_Num,4)
  + 30 * adj * pow(Loop,2) / pow(Stance_Num,3); 
  return stance_vel;
}

void PositionJointGroupController::command_assign(Angle_Ptr Angle)
{
	commands[0].position_ = Angle->lf.knee;
	commands[1].position_ = Angle->lf.hip;
	commands[2].position_ = Angle->lf.pitch;

	commands[3].position_ = Angle->rf.knee;
	commands[4].position_ = Angle->rf.hip;
	commands[5].position_ = Angle->rf.pitch;

  commands[6].position_ = Angle->lb.knee;
	commands[7].position_ = Angle->lb.hip;
	commands[8].position_ = Angle->lb.pitch;

	commands[9].position_ = Angle->rb.knee;
	commands[10].position_ = Angle->rb.hip;
	commands[11].position_ = Angle->rb.pitch;
}

void PositionJointGroupController::assign_next_foot_turn()
{
	switch(Leg_Order)
	{
		case LF:
		Pos_start = Pos_ptr->lf;
		Desired_Foot_Pos.assign(Pos_start.x*cos(-0.5)-Pos_start.y*sin(-0.5), Pos_start.x*sin(-0.5)+Pos_start.y*cos(-0.5), -Stance_Height);	
		break;
		case RF:
		Pos_start = Pos_ptr->rf;
		Desired_Foot_Pos.assign(Pos_start.x*cos(-0.5)-Pos_start.y*sin(-0.5), Pos_start.x*sin(-0.5)+Pos_start.y*cos(-0.5), -Stance_Height);	
		break;
		case LB:
		Pos_start = Pos_ptr->lb;
		Desired_Foot_Pos.assign(Pos_start.x*cos(-0.5)-Pos_start.y*sin(-0.5), Pos_start.x*sin(-0.5)+Pos_start.y*cos(-0.5), -Stance_Height);	
		break;
		case RB:
		Pos_start = Pos_ptr->rb;
		Desired_Foot_Pos.assign(Pos_start.x*cos(-0.5)-Pos_start.y*sin(-0.5), Pos_start.x*sin(-0.5)+Pos_start.y*cos(-0.5), -Stance_Height);	
		break;
		default:
		break;
	}	
}

void PositionJointGroupController::flow_control_turn(int timeorder)
{
	switch(timeorder)
	{
		case 0:
		if(Loop_Count>=30)
		{
			Loop_Count = 0;
			std::cout<<"Robot has been initialized! Please input 1 to continue:";
			std::cin>>Time_Order;	
			// Time_Order=1;
		}
		break;		

		case 1:
		if(Loop_Count>=30)
		{
			Loop_Count = 0;
			// std::cout<<"Imu data calibration done!"<<std::endl;
			Time_Order=2;
		}
		break;	

		case 2:
		if(Loop_Count>=Stance_Num)
		{
			Loop_Count = 0;			
			// std::cout<<"flow_control_backward: cog done"<<std::endl;
			Time_Order=4;				
			assign_next_foot_turn();
			
		}							
		break;			

		case 4:
		// if(foot_contact->isLegOnGround())
		if(Loop_Count>=Swing_Num)
		{
			foot_contact->clear();		
			Time_Order = 2;	
			if(Leg_Order==1)
			{
    	pose_init();
    	std::cout<<"pose_init down"<<std::endl;
      }
      Loop_Count = 0;				
			Leg_Order = (Leg_Order>1)?Leg_Order-2:3-Leg_Order;									
			// std::cout<<"flow_control_backward: swing done"<<std::endl;
			
    
		}					
		break;		
		default:break;
	}
}

void PositionJointGroupController::cog_adj_turn()
{
	_Position adj = {0,0,0};	
	if(Loop_Count<=1)
	{
		Cog_adj = get_CoG_adj_vec(Desired_Foot_Pos,Leg_Order); 
		// std::cout<<"CoG adjust vector:"<<Cog_adj.x<<", "<<Cog_adj.y<<std::endl;	
		
		Stance_Num = 40;		
		
		
			// Stance_Num = 20;
					
	}
	adj = get_stance_velocity(Cog_adj, Loop_Count);
	// cog_pos_assign(adj);
	reverse_kinematics();
}


}
// PLUGINLIB_DECLARE_CLASS(qr_control, PositionJointGroupController,
//                         qr_control::PositionJointGroupController,
//                         controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(qr_control::PositionJointGroupController,controller_interface::ControllerBase)

