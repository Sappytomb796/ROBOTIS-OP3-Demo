#ifndef OP3_MOTION_REPLAY_H
#define OP3_MOTION_REPLAY_H

#define REPLAY_START  1
#define REPLAY_LOAD   2
#define REPLAY_SAVE   3
#define REPLAY_PLAY   4
#define REPLAY_DELETE 5

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>

#include "robotis_controller_msgs/SetModule.h"

namespace robotis_op{
	class MotionReplay{
		public:
			MotionReplay();
			~MotionReplay();

		protected:
			const std::string MODULE_NAME;
			
			void enableModule(std::string module_name);

			void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
			void buttonCallback(const std_msgs::String::ConstPtr& msg);
			void webCallback(const std_msgs::MultiArrayDimension::ConstPtr& msg);

			void publishJointStates();

			bool saveReplay(std::string replay_name);
			bool loadReplay(std::string replay_name);
			void deleteReplay(std::string file_name);

			ros::NodeHandle nh_;
			
			ros::Subscriber op3_joints_sub_;
			ros::Subscriber web_sub_;
			ros::Subscriber button_sub_;	
			ros::Publisher joint_state_pub_;
			ros::Publisher module_pub_;	
			
			ros::ServiceClient joint_module_;
			
			std::vector<sensor_msgs::JointState> joint_states_;

			bool record_flag;
			std::map<std::string, double> joint_angles_;
	};

}

#endif
