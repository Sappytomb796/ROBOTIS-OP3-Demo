#ifndef OP3_MOTION_REPLAY_H
#define OP3_MOTION_REPLAY_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

namespace robotis_op{
    class MotionReplay{
        public:
            MotionReplay();
            ~MotionReplay();

        protected:
            const std::string MODULE_NAME;

            void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);

            ros::NodeHandle nh_;
            ros::Subscriber op3_joints_sub_;
			
			std::map<std::string, double> joint_angles_;
    };

}

#endif