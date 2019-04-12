#include <ros/ros.h>
#include "op3_motion_replay/op3_motion_replay.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_motion_replay_node");
  ros::NodeHandle nh;

  robotis_op::MotionReplay *op3_motion_replay = new robotis_op::MotionReplay();

  ROS_INFO("Start motion replay");

  ros::spin();

}