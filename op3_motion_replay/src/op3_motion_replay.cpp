#include "op3_motion_replay/op3_motion_replay.h"
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdio.h>

namespace robotis_op
{
	MotionReplay::MotionReplay()
		: nh_(ros::this_node::getName()),
		MODULE_NAME("op3_motion_replay")
	{
		op3_joints_sub_ = nh_.subscribe("/robotis/present_joint_states", 1,
										&MotionReplay::jointCallback, this);
		button_sub_ = nh_.subscribe("/robotis/open_cr/button", 1, &MotionReplay::buttonCallback, this);
		web_sub_ = nh_.subscribe("/robotis/replay/web", 1, &MotionReplay::webCallback, this);

		web_message_pub_ = nh_.advertise<std_msgs::String>("/robotis/replay/status", 0);
		joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);
		module_pub_ = nh_.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
				
		joint_states_.clear();
		
		record_flag = false;
		joint_angles_.clear();
	}

	MotionReplay::~MotionReplay()
	{
		
	}
	
	void MotionReplay::enableModule(std::string module_name)
	{
		std_msgs::String msg;
		msg.data = module_name;
		
		module_pub_.publish(msg);
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
			enableModule("direct_control_module");	// notifies framework to activate direct_control_module
			publishJointStates();
			ROS_INFO("Button: replay");
		}
	}

	void MotionReplay::webCallback(const std_msgs::MultiArrayDimension::ConstPtr& msg){
		switch(msg->size){
			case REPLAY_START:
				record_flag = !record_flag;
				ROS_INFO("Button: start");
				break;

			case REPLAY_SAVE:
				saveReplay(msg->label);
				ROS_INFO("Button: save");
				break;

			case REPLAY_LOAD:
				loadReplay(msg->label);
				ROS_INFO("Button: load");
				break;
			
			case REPLAY_DELETE:
				deleteReplay(msg->label);
				ROS_INFO("Button: delete");
				break;

			case REPLAY_PLAY:
				enableModule("direct_control_module"); // notifies framework to activate direct_control_module
				publishJointStates();
				ROS_INFO("Button: replay");
				break;

			default:
				ROS_INFO("Unrecognized button code received...");
				break;
		}
	}
	
	void MotionReplay::publishJointStates()
	{
		std_msgs::String web_message;
		std::string message;

		if(joint_states_.size() == 0){
			message = "Error: No replay to publish...";
			web_message.data = message;
			web_message_pub_.publish(web_message);
			ROS_INFO("%s", message.c_str());
			return;
		}
		
		ros::Rate loop_rate(30);

		for (std::vector<sensor_msgs::JointState>::const_iterator it = joint_states_.begin();
			 it != joint_states_.end(); ++it)
		{
			 joint_state_pub_.publish(*it);
			 loop_rate.sleep();
       		 ROS_INFO("%f: \n", (*it).position[0]);
		}
	}
	
	bool MotionReplay::saveReplay(std::string replay_name)
	{
		std_msgs::String web_message;
		std::string message;

		if(joint_states_.size() == 0)
		{
			message = "ERROR: No replay data to save...";
			web_message.data = message;
			web_message_pub_.publish(web_message);
			ROS_INFO("%s", message.c_str());
			return false;
		}
		
		ROS_INFO("Current path is %s", get_current_dir_name());
		std::ofstream file;
		file.open(replay_name + ".txt");
		
		if(!file.is_open())
		{
			message = "ERROR: failed to open " + replay_name + ".txt";
			web_message.data = message;
			web_message_pub_.publish(web_message);
			ROS_INFO("%s", message.c_str());
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
		
		message = replay_name + ".txt written.";
		web_message.data = message;
		web_message_pub_.publish(web_message);
		ROS_INFO("%s", message.c_str());
		return true;
	}
	
	bool MotionReplay::loadReplay(std::string replay_name)
	{
		std_msgs::String web_message;
		std::string message;

		std::ifstream file;
		file.open(replay_name + ".txt");
		
		if(!file.is_open())
		{
			message = "ERROR: failed to open " + replay_name + ".txt";
			web_message.data = message;
			web_message_pub_.publish(web_message);
			ROS_INFO("%s", message.c_str());
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
		message = replay_name + ".txt loaded.";
		web_message.data = message;
		web_message_pub_.publish(web_message);
		ROS_INFO("%s", message.c_str());
		return true;
	}

	void MotionReplay::deleteReplay(std::string file_name){
		std::string file_path = file_name + ".txt";
		std_msgs::String web_message;
		std::string message;
		
		if(remove(file_path.c_str()) != 0){
			message = "Removed " + file_path;
			web_message.data = message;
			web_message_pub_.publish(web_message);
			ROS_INFO("%s", message.c_str());
		}

		else{
			message = "Failed to remove " + file_path;
			web_message.data = message;
			web_message_pub_.publish(web_message);
			ROS_INFO("%s", message.c_str());
		}
	}
}
