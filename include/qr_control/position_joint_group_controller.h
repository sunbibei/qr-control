#ifndef QR_CONTROL_POSITION_JOINTGROUP_CONTROLLER_H_
#define QR_CONTROL_POSITION_JOINTGROUP_CONTROLLER_H_

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <queue>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/Float64MultiArray.h>
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

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>

#include "qr_control/ultility.h"
#include "qr_control/foot_contact.h"
#include "qr_control/swing.h"
#include "qr_control/math.h"
#include "qr_control/gesture_control.h"

namespace qr_control
{
	class PositionJointGroupController: public controller_interface::MultiInterfaceController<
	hardware_interface::PositionJointInterface,
	hardware_interface::ForceTorqueSensorInterface,
	hardware_interface::ImuSensorInterface>
	{
	public:
		PositionJointGroupController();
		~PositionJointGroupController();

		bool init(hardware_interface::RobotHW* robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);

		int n_joints_;
		int foot_sensor_const;
		FootContact* foot_contact;
		Swing* swing;
		Math* math;
		Gesture* gesture;
		
		Quartic Height;
		std::vector<double> commands;
		std_msgs::Float64MultiArray msg;
		std::vector< std::string > joint_names_;
		std::vector< hardware_interface::JointHandle > joints_;
		boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> joint_state_publisher_;

	private:
		int Loop_Count;
		int T_Count = 0;
		int Height_recovery_count = 0;
		bool Leg_On_Ground = 0;
		unsigned int Time_Order = 0;
		unsigned int Leg_Order = 2;
		double Init_Pos[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	// Angle Angle_Group = {{0,0,-0.547},{0,0,-0.547},{0,0,0.547},{0,0,0.547}};
		Angle Angle_Group = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
		
	Position Foot_Position_Group = {{0, 0, -L0 - L1 - L2},
	{0, 0, -L0 - L1 - L2},
	{0, 0, -L0 - L1 - L2},
	{0, 0, -L0 - L1 - L2}
};



Angle_Ptr Angle_ptr = &Angle_Group;
Position_Ptr Pos_ptr = &Foot_Position_Group;
_Position Desired_Foot_Pos = {0,0,0};
_Position Pos_start,Cog_adj;
std::vector<hardware_interface::ForceTorqueSensorHandle> td_handles_;
hardware_interface::ImuSensorHandle imu_handle_;


void hardware_delay_test();
void update_shoulder_pos(float pitch,float yaw,float roll);	
void forward_kinematics();
void update_angle();	
void stay_touch(int sensor);
void reverse_kinematics();
void pose_init();
void cog_adj();
void updownfuck();
void flow_control(int timeorder);
void cog_pos_assign(_Position Adj);
void swing_control();
void assign_next_foot();
void on_ground_control(int legId);	
void height_recovery(int legId,int sgn);
std::vector<double> vec_assign(Angle_Ptr Angle);

float get_adj_pos(float Adj, int t, int T);
_Position innerTriangle(const _Position &A, const _Position &B, const _Position &C);
_Position get_stance_velocity(_Position Adj_vec, unsigned int Loop);
_Position get_stance_acceration(_Position Adj_vec, unsigned int Loop);
_Position get_CoG_adj_vec(const _Position &Next_Foothold, unsigned int Swing_Order);
};

}
	
#endif