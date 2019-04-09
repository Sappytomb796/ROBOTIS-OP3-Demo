#include "robotis_op3_motion_replay/op3_motion_replay.h"

namespace robotis_op
{
	MotionReplay::MotionReplay()
		: nh_(ros::this_node::getName()),
		MODULE_NAME("op3_motion_replay")
	{
		op3_joints_sub_ = nh_.subscribe("/robotis/present_joint_states", 1, &MotionReplay::jointCallback, this);
		
		joint_angles_.clear();
	}

	MotionReplay::~MotionReplay()
	{
		
	}
	
	void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
	{
		if(msg.size() == 0)
			return;
		
		int num_states = msg.size();
		
		for(int i = 0; i < num_states; i++)
		{
			joint_angles_.insert(msg.name[i], msg.position[i]);
			
			// print for testing
			// might need to convert position (pos * 180 / M_PI)
			std::cout << msg.name[i] << " -> " << msg.position[i] << std::endl;
		}
	}
	
}