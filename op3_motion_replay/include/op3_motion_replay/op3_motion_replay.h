#ifndef OP3_MOTION_REPLAY_H
#define OP3_MOTION_REPLAY_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <vector>

namespace robotis_op{
    class MotionReplay{
    public:
        MotionReplay();
         ~MotionReplay();

    protected:
        const std::string MODULE_NAME;

        void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
	    void buttonCallback(const std_msgs::String::ConstPtr& msg);
	    
	    void publishJointStates();
    
	    bool saveReplay(std::string replay_name);
	    bool loadReplay(std::string replay_name);

        ros::NodeHandle nh_;
        ros::Subscriber op3_joints_sub_;
        ros::Subscriber button_sub_;	
        ros::Publisher joint_state_pub_;	
	    std::vector<sensor_msgs::JointState> joint_states_;
    	    
	    /*
	    std::vector<std::string> joint_name_;
	    std::map<std::string, std::vector<double>> joint_position_;
	    std::map<std::string, std::vector<double>> joint_velocity_;
	    std::map<std::string, std::vector<double>> joint_effort_;
	    */
            
        bool record_flag;
	    std::map<std::string, double> joint_angles_;
    };

}

#endif
