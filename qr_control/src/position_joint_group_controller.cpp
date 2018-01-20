#include "qr_control/position_joint_group_controller.h"

#define PRESS_GO do { LOG_WARNING << "Press any key to continue"; getchar(); } while(0);

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
  delete gesture;
}	

/**************************************************************************
 Description: Update Angles
**************************************************************************/
void PositionJointGroupController::UpdateImuData(const std_msgs::Float64MultiArray::ConstPtr& msg)
{				
	gesture->updateImuData(msg->data[0], msg->data[1], msg->data[2]);///57.3248
}

/**************************************************************************
   Description: initializition from robot_description
**************************************************************************/
bool PositionJointGroupController::init(hardware_interface::RobotHW* robot, ros::NodeHandle &n)
{		
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
  n_joints_ = joint_handles_.size();

  // std::string imu_label;
  // if (ros::param::get("imu", imu_label)) 
  // {
  // imu_handle_ = Label::getHardwareByName<middleware::ImuSensor>(imu_label);
  // if (nullptr == imu_handle_)
  //   LOG_ERROR << "The named '" << imu_label << "' qr.res.imu sensor does not exist!";
  // }
  // imu_ang_vel_ = imu_handle_->angular_velocity_const_pointer();

	for(int i=0; i<n_joints_; i++)
	{
		commands.push_back(0);
	}

	joint_state_publisher_.reset( new realtime_tools::RealtimePublisher<
    std_msgs::Float64MultiArray>(n, "/dragon/joint_commands", 10));

  walk_state_publisher_.reset( new realtime_tools::RealtimePublisher<
    std_msgs::String>(n, "/dragon/walk_states", 10));

	imu_sub = n.subscribe("/gesture/IMU", 10, &PositionJointGroupController::UpdateImuData,this);

	//foot contact process
	foot_contact = new FootContact();
	foot_contact->setThreshold(920, 900, 760, 850);
  foot_contact->setUpperThreshold(4000);
	foot_contact->init();

	swing = new Swing();
	swing->init();

	math = new Math();
	math->init();

	gesture = new Gesture();
	gesture->init();

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

  // std::cout<<"Please choose walk pattern: 0 means normal, 1 means hangup"<<std::endl;

  // std::cin>>HangUpWalk;
  // std::cout<<"Robot has been initialized! Please input transfer point to continue:";

  // std::cin>>Transfer_Point;

	std::cout<<"System Init Succeed!"<<std::endl;

	return true;
}
void PositionJointGroupController::__initAllofData() 
{
  jnt_poss_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  jnt_vels_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  jnt_tors_.reserve(LegType::N_LEGS * JntType::N_JNTS);

  auto jnt_manager = middleware::JointManager::instance();
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) 
  {
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW}) 
    {
      auto j = jnt_manager->getJointHandle(leg, jnt);
      jnt_poss_.push_back(j->joint_position_const_pointer());
      jnt_vels_.push_back(j->joint_velocity_const_pointer());
      jnt_tors_.push_back(j->joint_torque_const_pointer());
    }
  }

  // imu_ang_vel_ = imu_handle_->angular_velocity_const_pointer();
  // imu_lin_acc_ = imu_handle_->linear_acceleration_const_pointer();
  // imu_quat_    = imu_handle_->orientation_const_pointer();
}
/**************************************************************************
   Description: start the controller
**************************************************************************/
void PositionJointGroupController::starting(const ros::Time& time)
{
  LOG_WARNING << "STARTING";
	foot_contact->printThreshold();
}

/**************************************************************************
   Description: design state meachine: Adjust CoG <---> Switch Swing Leg
**************************************************************************/
void PositionJointGroupController::update(const ros::Time& time, const ros::Duration& period)
{    

  if(!Pause)  Run_State = true;  
  if(Run_State)
  {
    foot_contact->footForceDataUpdate(*(td_handles_[0].getForce()), *(td_handles_[1].getForce()),
                                     *(td_handles_[2].getForce()), *(td_handles_[3].getForce()));
  	Update_count++;
    //select gait pattern
    if(Update_count<2)  Turn_Walk = 3;//initial state;
    // if(Update_count==Transfer_Point)
    // {
    //   std::cout<<"Transfer point has reached! please input 1 to continue"<<std::endl;
    //   std::cin>>Turn_Walk;//left turn 1; right turn 5;
    //   // std::cin>>Back_Walk;
    //   // Back_Walk = 1;
    // }
    //end of the current move
    if(State_Transfer_Ready)
    {    
      if(Forw_Walk==1) Forw_Walk = 2;
      if(Back_Walk==1) Back_Walk = 2;
      if(Turn_Walk==1) Turn_Walk = 2;
      if(Turn_Walk==5) Turn_Walk = 6;
    }
    //state transfer
    if(Forw_Walk==2)
  	{
      //nothing;
      Forw_Walk = 3;
      Back_Walk = 0;
      Turn_Walk = 0;
    }
    if(Back_Walk==2)
    {
      Time_Order = 3;
      Forw_Walk = 0;
      Back_Walk = 3;
      Turn_Walk = 0;
    }
    if(Turn_Walk==2)//left
    {    
      //nothing
      Forw_Walk = 0;
      Back_Walk = 0;
      Turn_Walk = 3;
    }
    if(Turn_Walk==6)//right
    {
      //nothing
      Forw_Walk = 0;
      Back_Walk = 0;
      Turn_Walk = 7;
    }
  	if(!HangUpWalk)//contact_control
  	{
      foot_contact->tdEventDetect();   
      All_on_ground = contact_keep(foot_contact->contactStatus());       
    }
    /*********************************************************************************************/
    /*********************************Loop: 25 Hz*************************************************/
    /*********************************************************************************************/
  	count_loop++;   
  	if(count_loop>3)
  	{
  	  count_loop = 0;  
      State_Transfer_Ready = false;

  		switch (Time_Order)
  		{
  			case 0: //init gesture
        {
          pose_init();  
          flow_control(Time_Order);
          break;
        }			
  			case 1:	
        {
          Isstand_stable = turn_left(15);    

         //  if(Loop_Count>2) Isstand_stable = true;
    	    // if(!HangUpWalk)  
    	    // {
    	  		// foot_contact->tdEventDetect();
    	  	 // 	if(Loop_Count>3)
    	  	 // 	{
    			  // 	Isstand_stable = stand_stable(foot_contact->contactStatus());
    	  	 // 	}
    	  	 // 	if(Isstand_stable)
    	    //  	{
    	  	 // 		gesture->imuCalibration();
    	    //  	}
    	    // }
    	    // else
    	    // {
    			 	// Isstand_stable = true;	
    	    // }
    			flow_control(Time_Order);
    			break;
        }
  			case 2:
        {
          cog_adj();
          flow_control(Time_Order);
          break;
        }		
  			case 3://swing leg
        {
          SupportLeg[Leg_Order] = false;
          if(Lift_Off)
          {
            if(foot_contact->singleFootContactStatus(Leg_Order))
            {
              lift_ground_control(Leg_Order);
              assign_next_foot();
            }
            else
            {
              Lift_Off = false;
            }
          }
          if(!Lift_Off)
          {
            swing_control();
          } 
    			flow_control(Time_Order);		
    			break;
  		  }
  			default:break;
  	  }
  	  Loop_Count++;
  	}
    /**********************************************************************************************/
    /******************************Loop: 100 Hz****************************************************/
    /**********************************************************************************************/
  	command_assign(Angle_ptr);

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

    std::string s = "#";  
    //first part
    if(Forw_Walk==3)  s += "FW";
    if(Back_Walk==3)  s += "BW";
    if(Turn_Walk==3)  s += "TL";
    if(Turn_Walk==7)  s += "TR";
    s += "%";
    //second part
    if(State_Transfer_Ready) s += "E";
    else s+= "S";
    s += "%";
    //third part
    if(Time_Order==2) s += "C";
    else if(Time_Order==3) s += "S";
    else s += "N";
    s += "%";
    //fouth part
    if(SupportLeg[LF]) s += "S";
    else s += "A";
    if(SupportLeg[RF]) s += "S";
    else s += "A";
    if(SupportLeg[LB]) s += "S";
    else s += "A";
    if(SupportLeg[RB]) s += "S";
    else s += "A";
    s += "%";
    //fifth part
    if(Run_State) s += "R";
    else s += "S";
    s += "#";

    if(walk_state_publisher_ && walk_state_publisher_->trylock())
    {
      walk_state_publisher_->msg_.data.clear();
      for(int i=0; i<Joint_Num; i++)
      {
        walk_state_publisher_->msg_.data = s;
      }
      walk_state_publisher_->unlockAndPublish();
    }
  }
  // print_command();
  // std::cout<<":"<<std::endl;
  // std::cout<<"*****Loop Ends!*****"<<std::endl;
}

bool PositionJointGroupController::turn_left(float turn_angle)
{
  _Position tmp(0,0,0), adj(0,0,0);
  float num = 100;
  float beta = turn_angle/num;


  if(Trot_Phase==0 && Loop_Count>=num)  
  {
    std::cout<<"turn 10 degree already"<<std::endl;
    PRESS_GO;
    Trot_Phase = 1;
    Loop_Count = 0;
  }

  if(Trot_Phase==0)
  {
    tmp = math->cal_left_circul(beta,LF);
    Pos_ptr->lf = Pos_ptr->lf + tmp;
    tmp = math->cal_left_circul(beta,RF);
    Pos_ptr->rf = Pos_ptr->rf + tmp;
    tmp = math->cal_left_circul(beta,LB);
    Pos_ptr->lb = Pos_ptr->lb + tmp;
    tmp = math->cal_left_circul(beta,RB);
    Pos_ptr->rb = Pos_ptr->rb + tmp;
  }
  if(Trot_Phase==1)
  {    
    Stance_Num = 35;
    if(Loop_Count>Stance_Num) 
    {
      Trot_Phase = 2; 
      Loop_Count = 0;
      assign_next_foot();
      PRESS_GO;
    }
    else
    {
      if(Loop_Count<=1)
      {
        Cog_adj = get_CoG_adj_vec(Leg_Order); 
        // if(Leg_Order==3)
        // {
        //   Cog_adj.x+=3;
        //   Cog_adj.y+=3;
        // }
        if(Leg_Order==0)
        {
          Cog_adj.x -= 3;
          Cog_adj.y -= 5;
        }
      }
      adj = get_stance_velocity(Cog_adj, Loop_Count);
      cog_pos_assign(adj);      
    }    
  }
  if(Trot_Phase==2)
  {    
    Swing_Height = 6;
    TD_Height = 6;
    
    // if(Leg_Order==0)
    // {
    //   Swing_Height = 8;
    //   TD_Height = 8;
    // }
    foot_contact->tdEventDetect();
    SupportLeg[Leg_Order] = false;
    std::cout<<"Leg_Order: "<<Leg_Order<<" Lift_Off:"<<Lift_Off<<std::endl;
    if(Lift_Off)
    {
      if(foot_contact->singleFootContactStatus(Leg_Order))
      {
        lift_ground_control(Leg_Order);
        assign_next_foot();
      }
      else
      {
        PRESS_GO;
        Lift_Off = false;
      }
    }
    if(!Lift_Off)
    {
      swing_control();
    } 
    if(Leg_On_Ground) 
    {
      foot_contact->clear();    
      Trot_Phase = 1;
      Loop_Count = 0;
      for(int i=0;i<Leg_Num;i++)
      {
        SupportLeg[Leg_Order] = true;
      }
      foot_contact->clear();
      swing->init();
      Lift_Off = true;
      Leg_On_Ground = false;

      if(Leg_Order==1)
      {
        Trot_Phase = 3;        
      }
      Leg_Order = (Leg_Order>1) ? Leg_Order-2:3-Leg_Order;     
    }
  }  
  if(Trot_Phase==3)
  { 
    if(Loop_Count>Stance_Num) 
    { 
      Trot_Phase = 4;
      Loop_Count = 0;
      std::cout<<"END OF "<<std::endl;
      return true;
    }
    else
    {
      if(Loop_Count<=1)
      {
        Cog_adj = Pos_ptr->lb;            
      }
      adj = get_stance_velocity(Cog_adj, Loop_Count);  
      cog_pos_assign(adj);     
    }
  }
   std::cout<<"Position Command: lf: "<<Pos_ptr->lf.x<<" "<<Pos_ptr->lf.y<<" "<<Pos_ptr->lf.z<<"; ";
      std::cout<<"rf: "<<Pos_ptr->rf.x<<" "<<Pos_ptr->rf.y<<" "<<Pos_ptr->rf.z<<"; ";
      std::cout<<"lb: "<<Pos_ptr->lb.x<<" "<<Pos_ptr->lb.y<<" "<<Pos_ptr->lb.z<<"; ";
      std::cout<<"rb: "<<Pos_ptr->rb.x<<" "<<Pos_ptr->rb.y<<" "<<Pos_ptr->rb.z<<std::endl;
  reverse_kinematics();
  return false;
}


void PositionJointGroupController::print_command()
{
  // std::cout<<"Position Command: lf: "<<Pos_ptr->lf.x<<" "<<Pos_ptr->lf.y<<" "<<Pos_ptr->lf.z<<"; ";
  // std::cout<<"rf: "<<Pos_ptr->rf.x<<" "<<Pos_ptr->rf.y<<" "<<Pos_ptr->rf.z<<"; ";
  // std::cout<<"lb: "<<Pos_ptr->lb.x<<" "<<Pos_ptr->lb.y<<" "<<Pos_ptr->lb.z<<"; ";
  // std::cout<<"rb: "<<Pos_ptr->rb.x<<" "<<Pos_ptr->rb.y<<" "<<Pos_ptr->rb.z<<std::endl;

  std::cout<<"Joint Command: position: ";
  for(int i=0;i<Joint_Num;i++)
	{
		std::cout<<commands[i].position_<<" ";
	}	
}

bool PositionJointGroupController::stand_stable(std::vector<bool> IsContact)
{ 	
  int phase = 1, t = 100;
  float h = 0, mean = 0, beta = 0.05;  
  
  std::cout<<"stand stable:"<<IsContact[LF]<<" "<<IsContact[RF]<<" "
  	<<IsContact[LB]<<" "<<IsContact[RB]<<std::endl;
  foot_contact->printForce();

  if(IsContact[LF] && IsContact[RF] && IsContact[LB] &&IsContact[RB])
  {
    phase = 2;
  }
//phase one: make sure all legs are in touch with ground, using foot force"
	if(phase == 1)
	{
    beta = 0.05;
    std::cout<<"phase one: make sure all legs are in touch with ground"<<std::endl;
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
    beta = 0.01;
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
    beta = 0.01;
    std::cout<<"phase three: make sure height are close to desired height"<<std::endl;

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
  // std::cout<<"stable stand: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z
  // <<" "<<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;
 
  //phase three: record force data
  if(phase == 4)
  {  	
    foot_contact->setConst(foot_contact->getForceData(LF), foot_contact->getForceData(RF),
           foot_contact->getForceData(LB), foot_contact->getForceData(RB));
    foot_contact->printConst();
  	return true;
  }
  return false;
}

//after knowing which legs are all on ground, then under force limit recover height
bool PositionJointGroupController::contact_keep(std::vector<bool> IsContact)
{  
  float beta = 0.025, count = 0, leg_count = 0, h = 0;
  
  if(SupportLeg[LF])
  { 
    leg_count++;
    if(IsContact[LF]==false)
    {
      Pos_ptr->lf.z -= beta;      
    }  
    else if(fabs(Pos_ptr->lf.z-height[LF]) > beta)
    {
      Pos_ptr->lf.z -= Sgn(Pos_ptr->lf.z-height[LF]) * beta;  
      std::cout<<"contact_keep working! LF:"<<Pos_ptr->lf.z<<std::endl;       
    }
    else
    {
      count++;
    }
  }
  if(SupportLeg[RF])
  { 
    leg_count++;
    if(IsContact[RF]==false)
    {
      Pos_ptr->rf.z -= beta;      
    }   
    else if(fabs(Pos_ptr->rf.z-height[RF]) > beta)
    {
      Pos_ptr->rf.z -= Sgn(Pos_ptr->rf.z-height[RF]) * beta;   
      std::cout<<"contact_keep working! RF:"<<Pos_ptr->rf.z<<std::endl;          
    }
    else
    {
      count++;
    }
  }
  if(SupportLeg[LB])
  { 
    leg_count++;
    if(IsContact[LB]==false)
    {
      Pos_ptr->lb.z -= beta;      
    } 
    else if(fabs(Pos_ptr->lb.z-height[LB]) > beta)
    {
      Pos_ptr->lb.z -= Sgn(Pos_ptr->lb.z-height[LB]) * beta;
      std::cout<<"contact_keep working! LB:"<<Pos_ptr->lb.z<<std::endl;
    }
    else
    {
      count++;
    }
  }
  if(SupportLeg[RB])
  { 
    leg_count++;
    if(IsContact[RB]==false)
    {
      Pos_ptr->rb.z -= beta;      
    }   
    else if(fabs(Pos_ptr->rb.z-height[RB]) > beta)
    {
      Pos_ptr->rb.z -= Sgn(Pos_ptr->rb.z-height[RB]) * beta;
      std::cout<<"contact_keep working! RB:"<<Pos_ptr->rb.z<<std::endl;
    }
    else
    {
      count++;
    }
  }  

  reverse_kinematics(); 
  
  return ((count==leg_count) ? true:false);
}


void PositionJointGroupController::flow_control(int timeorder)
{
	switch(timeorder)
	{
		case 0:
    {
  		if(Loop_Count>=5)
  		{
  			Loop_Count = 0;
  			std::cout<<"Robot has been initialized! Please input 1 to continue:";
  			std::cin>>Time_Order;	
  			// Time_Order=1;
  		}
  		break;		
    }
		case 1:
    {
		  // if(Loop_Count>=30)
      if(Isstand_stable) 
  		{
  			foot_contact->clear();		
  			Loop_Count = 0;
        Trot_Phase = 0;
  			// std::cout<<"Imu data calibration done!"<<std::endl;
  			if(!HangUpWalk)
  			{
  				// std::cout<<"Robot has stand stable! Please input 2 to continue:"; 
       		// std::cin>>Time_Order; 
          std::cout<<"Keep Turning"; 
          PRESS_GO;
  			}
  			else
  			{
  				Time_Order=2;
  			} 
  		}
  		break;	
    }
		case 2:
    {
  		if(Loop_Count>=Stance_Num)
  		{
        State_Transfer_Ready = true;
  			Loop_Count = 0;			
  			// std::cout<<"flow_control: cog done"<<std::endl;
  			Time_Order=3;	      
        // std::cout<<"Robot moving CoG! Please input 3 to continue:";
        // std::cin>>Time_Order; 
  			assign_next_foot();		
  		}							
  		break;
    }	
		case 3:
    {
  		if((Leg_On_Ground && !HangUpWalk) || (Loop_Count>=Swing_Num && HangUpWalk))
  		{
  			Loop_Count = 0;
        for(int i=0;i<Leg_Num;i++)
        {
          SupportLeg[Leg_Order] = true;
        }
  			foot_contact->clear();
        swing->init();
        Lift_Off = true;
        Leg_On_Ground = false;
        State_Transfer_Ready = true;
  			Time_Order = 2;	      
        // std::cout<<"Robot swing done! Please input 2 to continue:";
        // std::cin>>Time_Order; 			
  			if(Forw_Walk==3) Leg_Order = (Leg_Order>1) ? Leg_Order-2:3-Leg_Order;		
        if(Back_Walk==3) Leg_Order = (Leg_Order>1) ? 3-Leg_Order:Leg_Order+2;
        if(Turn_Walk==3 || Turn_Walk==7) Leg_Order = (Leg_Order>1) ? Leg_Order-2:3-Leg_Order;   
  		}					
  		break; 
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

void PositionJointGroupController::lift_ground_control(int legId)
{
  double err = -0.1;  
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
   Description: Initialization. 
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

_Position PositionJointGroupController::innerTriangle(
	const _Position &A, const _Position &B, const _Position &C)
{	
  _Position innerheart, a_inner, b_inner, c_inner, first_cross, second_cross;
  _Position cog_init(0,0,0);
  float ratio = 1;
  float radius = math->inscribedCircleRadius(A,B,C);
  
  innerheart = math->getInnerHeart(A, B, C);

  if(radius>cogThreshold)
  {
    ratio = (radius-cogThreshold)/radius; 
    
    a_inner = math->formulaLineSection(A, innerheart, ratio);
    b_inner = math->formulaLineSection(B, innerheart, ratio);   
    c_inner = math->formulaLineSection(C, innerheart, ratio);
    
    first_cross = math->getCrossPoint(a_inner,b_inner,cog_init,innerheart);
    second_cross = math->getCrossPoint(a_inner,c_inner,cog_init,innerheart);    
   
    if(first_cross.x<=max(a_inner.x, b_inner.x) && first_cross.x>=min(a_inner.x, b_inner.x))
    {          
      return first_cross;
    }
    if(second_cross.x<=max(a_inner.x, c_inner.x) && second_cross.x>=min(a_inner.x, c_inner.x))
    {      
      return second_cross;
    }   
  }
  else
  {
    return innerheart;    
  }
}
/**********************************************************
Author: WangShanren
Date: 2017.2.22
Description: calculating CoG position and CoG adjust vector,
using next foothold,while 1 means to right side;
Input: four feet(end effector) position(x,y,z);
       Swing leg order and next Swing leg position
Output: CoG adjust vector
***********************************************************/
_Position PositionJointGroupController::get_CoG_adj_vec(
	unsigned int Swing_Order)
{
	_Position crosspoint;
	switch (Swing_Order)
	{
		case LF:		
		case LB:
		crosspoint = math->getCrossPoint(Pos_ptr->lf+gesture->getSingleShoulderPos(LF),
			Pos_ptr->rb+gesture->getSingleShoulderPos(RB),
			Pos_ptr->rf+gesture->getSingleShoulderPos(RF), 
			Pos_ptr->lb+gesture->getSingleShoulderPos(LB));
		return innerTriangle(crosspoint, Pos_ptr->rf+gesture->getSingleShoulderPos(RF), 
			Pos_ptr->rb+gesture->getSingleShoulderPos(RB));		
		case RF:
		case RB:
		crosspoint = math->getCrossPoint(Pos_ptr->rf+gesture->getSingleShoulderPos(RF), 
			Pos_ptr->lb+gesture->getSingleShoulderPos(LB),
			Pos_ptr->lf+gesture->getSingleShoulderPos(LF), 
			Pos_ptr->rb+gesture->getSingleShoulderPos(RB));
		return innerTriangle(crosspoint, Pos_ptr->lf+gesture->getSingleShoulderPos(LF), 
			Pos_ptr->lb+gesture->getSingleShoulderPos(LB));		
	}
}

void PositionJointGroupController::assign_next_foot()
{  
  if(Turn_Walk>=2 && Turn_Walk<5) 
  {
    Desired_Foot_Pos = Pos_ptr->lb;//left
    Desired_Foot_Pos.z = -Stance_Height + TD_Height;
  }
      
  switch(Leg_Order)
	{
		case LF:
    {
  		Pos_start = Pos_ptr->lf;
      if(Forw_Walk>=2) Desired_Foot_Pos.assign( Foot_Steps, 6, Pos_start.z+TD_Height);	
      if(Back_Walk>=2) Desired_Foot_Pos.assign(-Foot_Steps, 6, Pos_start.z+TD_Height);  
      
  		break;
    }
		case RF:
    {
  		Pos_start = Pos_ptr->rf;
  		if(Forw_Walk>=2) Desired_Foot_Pos.assign( Foot_Steps, -6, Pos_start.z+TD_Height);	
      if(Back_Walk>=2) Desired_Foot_Pos.assign(-Foot_Steps, -6, Pos_start.z+TD_Height);          
  		break;
    }
		case LB:
    {
  		Pos_start = Pos_ptr->lb;
  		if(Forw_Walk>=2) Desired_Foot_Pos.assign( Foot_Steps, 6, Pos_start.z+TD_Height);	
      if(Back_Walk>=2) Desired_Foot_Pos.assign(-Foot_Steps, 6, Pos_start.z+TD_Height);  
      if(Turn_Walk>=2 && Turn_Walk<5) Desired_Foot_Pos.assign(0, 0, Pos_start.z+TD_Height);
  		break;
    }
		case RB:
    {
  		Pos_start = Pos_ptr->rb;
  		if(Forw_Walk>=2) Desired_Foot_Pos.assign( Foot_Steps, -6, Pos_start.z+TD_Height);	
      if(Back_Walk>=2) Desired_Foot_Pos.assign(-Foot_Steps, -6, Pos_start.z+TD_Height);  
  		break;
    }
		default:break;
	}	
}

void PositionJointGroupController::cog_adj()
{
	// Switch = 1;
	_Position adj = {0,0,0};
	
	if(Loop_Count<=1)
	{
		Cog_adj = get_CoG_adj_vec(Leg_Order);     
    
		// std::cout<<"CoG adjust vector:"<<Cog_adj.x<<", "<<Cog_adj.y<<std::endl;		
		if(Forw_Walk)
    {
      Cog_adj.x += 2;
      Stance_Num = (Leg_Order<2) ? 1:25;  
    } 
    if(Back_Walk)	
    {
      Cog_adj.x -= 2;
      Stance_Num = (Leg_Order<2) ? 25:1; 
    }
    if(Turn_Walk) 
    {
      Stance_Num = (Leg_Order<2) ? 1:25;  
    }
	}

	adj = get_stance_velocity(Cog_adj, Loop_Count);
  // std::cout<<"cog_adj h_adj_step:";
  

	cog_pos_assign(adj);
	reverse_kinematics();
  // std::cout<<"cogad: Pos_ptr.z:"<<Pos_ptr->lf.z<<" "<<Pos_ptr->rf.z<<" "
  // <<Pos_ptr->lb.z<<" "<<Pos_ptr->rb.z<<" "<<std::endl;

}

void PositionJointGroupController::swing_control()
{
  if(Pos_start == Desired_Foot_Pos)
  {
    Leg_On_Ground = true;
    return;
  }

	_Position s = {0,0,0};
  Leg_On_Ground = false;
  SupportLeg[Leg_Order] = false;   

	if(Loop_Count<=Swing_Num)
	{					
		if(Loop_Count>Swing_Num/3*2)
		{
			Leg_On_Ground = foot_contact->singleFootContactStatus(Leg_Order);	
		}
		if(!Leg_On_Ground)
		{
			s = swing->get_rect_pos(Pos_start, Desired_Foot_Pos, Loop_Count, Swing_Num, Swing_Height);
					
      switch(Leg_Order)
      {
        case LF:
          Pos_ptr->lf = Pos_start + s;
        break;
        case RF:
          Pos_ptr->rf = Pos_start + s;
        break;
        case LB:
          Pos_ptr->lb = Pos_start + s;
        break;
        case RB:
          Pos_ptr->rb = Pos_start + s;
        break;
        default:break;
      }       
		}		
	}
	else
	{	
	  Leg_On_Ground = foot_contact->singleFootContactStatus(Leg_Order);		
		if(!Leg_On_Ground)
		{
			on_ground_control(Leg_Order);
			// std::cout<<"ON ground_control is working"<<std::endl;
		}			
	}	
  if(Leg_On_Ground)
  {
    std::cout<<"Swing ENDS: Loop_Count:"<<Loop_Count<<" Swing_Num"<<Swing_Num<<std::endl;
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
		std::cout<<"Loop:"<<Loop<<" Stance_Num:"<<Stance_Num<<std::endl;
		ROS_ERROR("Time Order wrong while stance");
	}

	_Position stance_vel = {0,0,0};
	stance_vel.x = 30 * Adj_vec.x * pow(Loop,4) / pow(Stance_Num,5)
	- 60 * Adj_vec.x * pow(Loop,3) / pow(Stance_Num,4)
	+ 30 * Adj_vec.x * pow(Loop,2) / pow(Stance_Num,3);
	stance_vel.y = 30 * Adj_vec.y * pow(Loop,4) / pow(Stance_Num,5)
	- 60 * Adj_vec.y * pow(Loop,3) / pow(Stance_Num,4)
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
//
void PositionJointGroupController::print_error()
{
  // std::cout<<"command,state,error: ";
  for(int i=0;i<Joint_Num;i++)
  {
    std::cout<<commands[i].position_<<","<<*(jnt_poss_[i])<<","<<fabs(commands[i].position_ - *(jnt_poss_[i]))<<", ";
  }  
  std::cout<<std::endl;
}


}
// PLUGINLIB_DECLARE_CLASS(qr_control, PositionJointGroupController,
//                         qr_control::PositionJointGroupController,
//                         controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(qr_control::PositionJointGroupController,controller_interface::ControllerBase)

