#include "op3_motion_replay/op3_motion_replay.h"
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace robotis_op
{
	MotionReplay::MotionReplay()
		: nh_(ros::this_node::getName()),
		MODULE_NAME("op3_motion_replay")
	{
		op3_joints_sub_ = nh_.subscribe("/robotis/present_joint_states", 1,
										&MotionReplay::jointCallback, this);
		button_sub_ = nh_.subscribe("/robotis/open_cr/button", 1, &MotionReplay::buttonCallback, this);
		joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);
		joint_states_.clear();
		
		record_flag = false;
		joint_angles_.clear();
	}

	MotionReplay::~MotionReplay()
	{
		
	}
	
	void MotionReplay::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
	{
		if(!record_flag)
			return;
			
		if(msg->name.size() == 0)
			return;
		
		int num_states = msg->name.size();
		
		sensor_msgs::JointState new_msg;
		
		for(int i = 0; i < num_states; i++)
		{
			new_msg.name.push_back(msg->name[i]);
			new_msg.position.push_back(msg->position[i]);
			new_msg.velocity.push_back(msg->velocity[i]);
			new_msg.effort.push_back(msg->effort[i]);
			
			// print for testing
			// might need to convert position (pos * 180 / M_PI)
			ROS_INFO("%s\nposition: %f\nvelocity: %f\neffort: %f\n",
					 msg->name[i].c_str(), msg->position[i], msg->velocity[i], msg->effort[i]);
		}
		
		joint_states_.push_back(new_msg);
	}

	void MotionReplay::buttonCallback(const std_msgs::String::ConstPtr& msg){
		if(msg->data == "start"){
			record_flag = !record_flag;
			ROS_INFO("Button: start");
		}

		if(msg->data == "user"){
			saveReplay("test");
			ROS_INFO("Button: save");
		}
		if(msg->data == "mode"){
			publishJointStates();
			ROS_INFO("Button: replay");
		}
	}
	
	void MotionReplay::publishJointStates()
	{
		if(joint_states_.size() == 0){
			ROS_INFO("Error: No replay to publish...");
			return;
		}
			
		for (std::vector<sensor_msgs::JointState>::const_iterator it = joint_states_.begin();
			 it != joint_states_.end(); ++it)
		{
			 joint_state_pub_.publish(*it);
		}
	
	}
	
	bool MotionReplay::saveReplay(std::string replay_name)
	{
		if(joint_states_.size() == 0)
		{
			ROS_INFO("ERROR: no replay data to save...");
			return false;
		}
		
		ROS_INFO("Current path is %s", get_current_dir_name());
		std::ofstream file;
		file.open(replay_name + ".txt");
		
		if(!file.is_open())
		{
			ROS_INFO("ERROR: failed to open %s.txt", replay_name.c_str());
			return false;
		}
		
		for (std::vector<sensor_msgs::JointState>::const_iterator it = joint_states_.begin();
			 it != joint_states_.end(); ++it)
		{
			for (std::vector<double>::const_iterator pit = (*it).position.begin();
				 pit != (*it).position.end(); ++pit)
				file << *pit << '\t';
			file << '\n';
		}
		
		file.close();
		
		ROS_INFO("%s.txt written.", replay_name.c_str());
		return true;
	}
	
	bool MotionReplay::loadReplay(std::string replay_name)
	{
		std::ifstream file;
		file.open(replay_name);
		
		if(!file.is_open())
		{
			std::cout << "ERROR: failed to open " << replay_name << ".txt\n";
			return false;
		}
		
		std::string data;
		std::string token;
		char delimiter = '\t';
		sensor_msgs::JointState msg;
		
		joint_states_.clear();
		
		while(getline(file, data))
		{
			std::stringstream stream(data);
			
			while(getline(stream, token, delimiter))
			{
				msg.position.push_back(atof(token.c_str()));
				ROS_INFO("%s\n", token.c_str());
			}
			
			joint_states_.push_back(msg);
			msg.name.clear();
			msg.position.clear();
		}
		
		file.close();
		
		return true;
	}
}
