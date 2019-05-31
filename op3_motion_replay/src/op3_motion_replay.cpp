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
		web_sub_ = nh_.subscribe("/robotis/replay/web", 1, &MotionReplay::webCallback, this);

		web_message_pub_ = nh_.advertise<std_msgs::String>("/robotis/replay/status", 0);
		joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
		module_pub_ = nh_.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);

		joint_states_.clear();	
		record_flag = false;
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

		new_msg.header = msg->header;

		for(int i = 0; i < num_states; i++)
		{
			new_msg.name.push_back(msg->name[i]);
			new_msg.position.push_back(msg->position[i]);
			new_msg.velocity.push_back(msg->velocity[i]);
			new_msg.effort.push_back(msg->effort[i]);

			ROS_INFO("%s\nframe_id: %s\nposition: %f\nvelocity: %f\neffort: %f\n",
					msg->header.frame_id.c_str(), msg->name[i].c_str(), msg->position[i], msg->velocity[i], msg->effort[i]);
		}

		joint_states_.push_back(new_msg);
	}

	void MotionReplay::webCallback(const std_msgs::MultiArrayDimension::ConstPtr& msg){
		switch(msg->size){
			case REPLAY_START:
				if(record_flag == false)
					joint_states_.clear();

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
				enableModule("none"); 
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
		file.open(replay_name + ".mrf");

		if(!file.is_open())
		{
			message = "ERROR: failed to open " + replay_name + ".mrf";
			web_message.data = message;
			web_message_pub_.publish(web_message);
			ROS_INFO("%s", message.c_str());
			return false;
		}

		for (std::vector<sensor_msgs::JointState>::const_iterator it = joint_states_.begin();
				it != joint_states_.end(); ++it)
		{
			int len = (*it).name.size();
			for(int i = 0; i < len; i++)
			{
				file << (*it).name[i] << ' ';
				file << (*it).position[i] << ' ';
				file << (*it).velocity[i] << ' ';
				file << (*it).effort[i] << '\t';
			}

			file << '\n';
		}

		file.close();

		message = replay_name + ".mrf written.";
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
		file.open(replay_name + ".mrf");

		if(!file.is_open())
		{
			message = "ERROR: failed to open " + replay_name + ".mrf";
			web_message.data = message;
			web_message_pub_.publish(web_message);
			ROS_INFO("%s", message.c_str());
			return false;
		}

		std::string data;
		std::string token;
		std::string state;
		char delimiter_tab = '\t';
		char delimiter_space = ' ';
		sensor_msgs::JointState msg;

		joint_states_.clear();

		while(getline(file, data))
		{
			std::stringstream stream(data);

			while(getline(stream, state, delimiter_tab))
			{
				std::stringstream stream_state(state);

				getline(stream_state, token, delimiter_space);
				msg.name.push_back(token);

				getline(stream_state, token, delimiter_space);
				msg.position.push_back(atof(token.c_str()));

				getline(stream_state, token, delimiter_space);
				msg.velocity.push_back(atof(token.c_str()));

				getline(stream_state, token);
				msg.effort.push_back(atof(token.c_str()));
			}

			joint_states_.push_back(msg);
			msg.name.clear();
			msg.position.clear();
			msg.velocity.clear();
			msg.effort.clear();
		}

		file.close();
		message = replay_name + ".mrf loaded.";
		web_message.data = message;
		web_message_pub_.publish(web_message);
		ROS_INFO("%s", message.c_str());
		return true;
	}

	void MotionReplay::deleteReplay(std::string file_name){
		std::string file_path = file_name + ".mrf";
		std_msgs::String web_message;
		std::string message;

		if(remove(file_path.c_str()) == 0){
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
