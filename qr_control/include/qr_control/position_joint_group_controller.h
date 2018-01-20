#ifndef QR_CONTROL_POSITION_JOINTGROUP_CONTROLLER_H_
#define QR_CONTROL_POSITION_JOINTGROUP_CONTROLLER_H_

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <time.h>
#include <queue>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <urdf/model.h>
#include <angles/angles.h>
#include <pthread.h>
#include <pluginlib/class_list_macros.h>
#include <repository/resource/imu_sensor.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>

#include "repository/resource/joint_manager.h"
#include "qr_control/ultility.h"
#include "qr_control/foot_contact.h"
#include "qr_control/swing.h"
#include "qr_control/math.h"
#include "qr_control/gesture_control.h"


namespace qr_control
{
	class PositionJointGroupController: public controller_interface::MultiInterfaceController<
	hardware_interface::ForceTorqueSensorInterface,
	hardware_interface::ImuSensorInterface>
	{
	public:
		PositionJointGroupController();
		~PositionJointGroupController();

		bool init(hardware_interface::RobotHW*, ros::NodeHandle &n) override;
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void UpdateImuData(const std_msgs::Float64MultiArray::ConstPtr& msg);
		ros::Subscriber imu_sub;
		
		float test_tmp;
		int n_joints_;
		int foot_sensor_const;
		FootContact* foot_contact;
		Swing* swing;
		Math* math;
		Gesture* gesture;
		
		Quartic Height;
		// std::vector<double> commands;
		std_msgs::Float64MultiArray msg;
		std::vector< std::string > joint_names_;
		std::vector< hardware_interface::JointHandle > joints_;
		boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> joint_state_publisher_;
				boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::String>> walk_state_publisher_;

	private:
    /***********************/
 		bool HangUpWalk = false;
 		bool Run_State = false;
 		bool Pause = false;
 		bool Only_CoG = false;
 		bool Lift_Off = true;
 		bool State_Transfer_Ready = false;
 		int Forw_Walk = 0;
 		int Back_Walk = 0;
 		int Turn_Walk = 0;
 		int Trot_Phase = 0;
 		/***********************/
		int Loop_Count;
		long long Update_count = 0;
		long long Error_count = 0;
		int Transfer_Point = 0;
		int Switch = 0;
		int count_loop = 0;
		bool Leg_On_Ground = 0;
		unsigned int Time_Order = 0;
		unsigned int Leg_Order = 2;
		double Init_Pos[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	// Angle Angle_Group = {{0,0,-0.547},{0,0,-0.547},{0,0,0.547},{0,0,0.547}};
		Angle Angle_Group = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};		
		
		Position Foot_Position_Group = {{0, 0, -L0 - L1 - L2},
		{0, 0, -L0 - L1 - L2},
		{0, 0, -L0 - L1 - L2},
		{0, 0, -L0 - L1 - L2}};

		Position Foot_State_Position = {{0, 0, -L0 - L1 - L2},
		{0, 0, -L0 - L1 - L2},
		{0, 0, -L0 - L1 - L2},
		{0, 0, -L0 - L1 - L2}};

		bool Isstand_stable;
		bool All_on_ground;
// std::vector<bool> IsContact;
		std::vector<float> height;
		std::vector<bool> SupportLeg;

		std::vector<Commands> commands;
		Angle_Ptr Angle_ptr = &Angle_Group;
		Position_Ptr Pos_ptr = &Foot_Position_Group;
		Position_Ptr State_Pos = &Foot_State_Position;
		_Position Desired_Foot_Pos = {0,0,0};
		_Position Pos_start,Cog_adj;
		_Position swing_adj_CoG;

		std::vector<middleware::Joint*>             joint_handles_;
		std::vector<hardware_interface::ForceTorqueSensorHandle> td_handles_;
		middleware::ImuSensor*                      imu_handle_;

		const double*                  imu_ang_vel_;
		const double*                  imu_lin_acc_;
		const double*                  imu_quat_;

		std::vector<const double*>     jnt_poss_;
		std::vector<const double*>     jnt_vels_;
		std::vector<const double*>     jnt_tors_;
		void __initAllofData();

		void update_shoulder_pos(float pitch,float yaw,float roll);	
		void forward_kinematics();
		void reverse_kinematics();
		void pose_init();
		void cog_adj();
		bool turn_left(float turn_angle);
		void flow_control(int timeorder);
		void cog_pos_assign(_Position Adj);
		void cog_swing_assign(_Position Adj, int legId);
		void swing_control();
		void assign_next_foot();
		void on_ground_control(int legId);	
		void lift_ground_control(int legId);
		void command_assign(Angle_Ptr Angle);
		void print_command();
		void print_error();

		bool stand_stable(std::vector<bool> IsContact);
		bool contact_keep(std::vector<bool> IsContact);

		_Position innerTriangle(const _Position &A, const _Position &B, const _Position &C);
		_Position get_stance_velocity(_Position Adj_vec, unsigned int Loop);
		float get_stance_velocity(float adj, unsigned int Loop);

		_Position get_CoG_adj_vec(unsigned int Swing_Order);
	};

}

#endif