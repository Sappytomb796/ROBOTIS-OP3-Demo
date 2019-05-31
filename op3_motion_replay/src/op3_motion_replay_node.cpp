#include <ros/ros.h>
#include "op3_motion_replay/op3_motion_replay.h"

bool isManagerRunning();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_motion_replay_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(30);

  robotis_op::MotionReplay *op3_motion_replay = new robotis_op::MotionReplay();
  
  while(ros::ok()){
    ros::Duration(1.0).sleep();
    if(isManagerRunning() == true){
      break;
      ROS_INFO("Connected to op3_manager.");
    }
    ROS_WARN("Waiting for op3_manager...");
  }

  ROS_INFO("Start motion replay");

  while(ros::ok()){
  	ros::spinOnce();
	loop_rate.sleep();
  }

  return 0;
}

bool isManagerRunning(){
  std::vector<std::string> nodes;
  ros::master::getNodes(nodes);
  std::string manager_name = "/op3_manager";

  for (unsigned int nodes_idx = 0; nodes_idx < nodes.size(); nodes_idx++){
    if(nodes[nodes_idx] == manager_name)
      return true;
  }

  return false;
}
