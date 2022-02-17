/*
 *  joystick_planner.cpp
 *
 *  author: Markus Erik Sügis <markus.sugis@gmail.com>
 */

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

class SpeedControllerJSP {

private:
    ros::Subscriber raw_cmd_vel_sub;
    ros::Subscriber modifier_sub;
	ros::Publisher filtered_cmd_vel_pub;
	
	std::string cmd_vel_sub_topic;
	std::string cmd_vel_pub_topic;
	std::string speed_modifier_sub_topic;
	double max_vel_x;
	double min_vel_x;
	double speed_modifier;

	
public:
	
    SpeedControllerJSP(ros::NodeHandle &n) {
        ros::NodeHandle private_nh("~");
		
		// Set parameters for node
        private_nh.param<std::string>("cmd_vel_sub_topic", this->cmd_vel_sub_topic, "move_base_flex/cmd_vel");
        private_nh.param<std::string>("cmd_vel_pub_topic", this->cmd_vel_pub_topic, "cmd_vel");
        private_nh.param<std::string>("speed_modifier_sub_topic", this->speed_modifier_sub_topic, "joystick_planner/speed_modifier");
        private_nh.param("max_vel_x", this->max_vel_x, 0.5);
        private_nh.param("min_vel_x", this->min_vel_x, 0.1);
		
		// subscribe to  '/joy' topic to which the Joy node publishes controller input values
        this->raw_cmd_vel_sub = n.subscribe<geometry_msgs::Twist>(this->cmd_vel_sub_topic, 30, &SpeedControllerJSP::translate_cmd_vel, this);
        this->modifier_sub = n.subscribe<std_msgs::Float32>(this->speed_modifier_sub_topic, 30, &SpeedControllerJSP::update_speed_modifier, this);
		// subscribe to '/odometry/filtered' topic where the robot's odometry is published to
		this->filtered_cmd_vel_pub = n.advertise<geometry_msgs::Twist>(this->cmd_vel_pub_topic, 30);
    }
    
    void update_speed_modifier(const std_msgs::Float32::ConstPtr &speed)
    {
    	this->speed_modifier = speed->data;
    }
    
    void translate_cmd_vel(const geometry_msgs::Twist::ConstPtr &cmd)
    {
    	if (cmd->linear.x > 0)
    	{
			geometry_msgs::Twist newCmd;
			newCmd.angular = cmd->angular;
			newCmd.linear = cmd->linear;
			
			if (this->speed_modifier > this->max_vel_x)
			{
				newCmd.linear.x = this->max_vel_x;
			}
			else if (this->speed_modifier < this->min_vel_x)
			{
				newCmd.linear.x = this->min_vel_x;
			}
			else
			{
				newCmd.linear.x = this->speed_modifier;
			}
			this->filtered_cmd_vel_pub.publish(newCmd);
    	}
    	else
    	{
    		this->filtered_cmd_vel_pub.publish(cmd);
    	}
    }
};

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "SpeedControllerJSP");
    ros::NodeHandle n;

    // Create planner object
    SpeedControllerJSP *jsp = new SpeedControllerJSP(n);
	
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete jsp;
	return 0;
}