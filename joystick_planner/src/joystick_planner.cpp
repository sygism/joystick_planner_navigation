/*
 *  joystick_planner.cpp
 *
 *  author: Markus Erik Sügis <markus.sugis@gmail.com>
 */

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <math.h>

class JoystickPlanner {
	
	struct Euler{
		double roll, pitch, yaw;
	};
	
	typedef enum{
		button_up,
		button_rising,
		button_falling,
		button_down,
	} Button_State;

private:
    ros::Subscriber controller_raw;
	ros::Subscriber robot_odometry;
	ros::Publisher speed_modifier_pub;
	ros::Publisher input_status_pub;
	
	// Variable for determining whether input is enabled
	bool inputEnabled = false;
	
    // variables for raw data from the joy node
    double leftStickY, leftStickX, rightStickY, rightStickX;
    int xButton, l1;
	
	// variables for the robot's position
	double robot_x_pos, robot_y_pos;
	
	// variable for the robot's orientation represented by Euler angles
	Euler robot_orientation;

    // initialize variables for linear and angular scales
    double scale_linear, scale_angular;

    // variables for the filtered data
    double filt_leftStickX, filt_leftStickY, filt_rightStickX, filt_rightStickY, filt_r2, filt_l2;
	
	// create SimpleActionClient for sending constructed paths to 'move_base_flex'
	actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* pc;
	
	// Odometry node passed as command line argument
	std::string odom_node;
	// Parameter for specifying type of controller used
	std::string controller_mapping;
	// Parameter for specifying the robot's base frameSpeed
	std::string robot_base_frame;
	// Parameter for specifying the local planner used
	std::string local_planner;
	// Parameter for specifying path execution topic
	std::string exe_path_topic;
	// Parameter for specifying input status topic
	std::string input_status_topic;
	// Parameter for specifying robot's speed topic
	std::string speed_modifier_pub_topic;
	
	// Button debounce stuff
	Button_State l1_btn_state;
	
public:
	
    JoystickPlanner(ros::NodeHandle &n) {
        ros::NodeHandle private_nh("~");
		
		// set linear and angular scales for joysticks
        private_nh.param("scale_linear", this->scale_linear, 1.0);
        private_nh.param("scale_angular", this->scale_angular, 1.0);
        private_nh.param<std::string>("odom_node", this->odom_node, "odom");
        private_nh.param<std::string>("controller_type", this->controller_mapping, "DS4");
        private_nh.param<std::string>("robot_base_frame", this->robot_base_frame, "base_link");
        private_nh.param<std::string>("local_planner", this->local_planner, "teb_local_planner/TebLocalPlannerROS");
        private_nh.param<std::string>("path_exe_topic", this->exe_path_topic, "move_base_flex/exe_path");
        private_nh.param<std::string>("input_status_topic", this->input_status_topic, "joystick_planner/input_status");
        private_nh.param<std::string>("speed_modifier_topic", this->speed_modifier_pub_topic, "joystick_planner/speed_modifier");
        private_nh.param("move_base_flex/exe_path/actionlib_client_sub_queue_size", 1);
        
        std::string msg = "JoystickPlanner INFO:\n"
        		"Parameters**:\n"
        		"JoystickPlanner/odom_node: " + this->odom_node + "\n"
				"JoystickPlanner/controller_type: " + this->controller_mapping + "\n"
				"JoystickPlanner/robot_base_frame: " + this->robot_base_frame + "\n"
				"JoystickPlanner/local_planner: " + this->local_planner + "\n"
				"JoystickPlanner/path_exe_topic: " + this->exe_path_topic + "\n";
        ROS_INFO("%s", msg.c_str());
		
		// subscribe to  '/joy' topic to which the Joy node publishes controller input values
        this->controller_raw = n.subscribe<sensor_msgs::Joy>("/joy", 10, &JoystickPlanner::mapOutput, this);
		// subscribe to '/odometry/filtered' topic where the robot's odometry is published to
		this->robot_odometry = n.subscribe<nav_msgs::Odometry>(this->odom_node, 10,  &JoystickPlanner::updateRobotPosition, this);
		// Create publisher for joystick given robot linear speed
		this->speed_modifier_pub = n.advertise<std_msgs::Float32>(this->speed_modifier_pub_topic, 30);
		// Create publisher for input enabled info
		this->input_status_pub = n.advertise<std_msgs::Bool>(this->input_status_topic, 30);
		
		pc = new actionlib::SimpleActionClient<mbf_msgs::ExePathAction>(this->exe_path_topic, true);
		// wait for server to start
		pc->waitForServer();
		
        ROS_INFO("JoystickPlanner INFO: scale_linear set to: %f", this->scale_linear);
        ROS_INFO("JoystickPlanner INFO: scale_angular set to: %f", this->scale_angular);
        ROS_INFO("JoystickPlanner INFO: JoystickPlanner initialized.");
        ROS_INFO("JoystickPlanner INFO: Controller input is now disabled.");
    }
	
	void updateRobotPosition(const nav_msgs::Odometry::ConstPtr &pos)
	{
		// fetch the robot's position in the world
		this->robot_x_pos = pos->pose.pose.position.x;
		this->robot_y_pos = pos->pose.pose.position.y;
		// fetch the robot's orientation as a quaternion and convert it to Euler angles
		this->robot_orientation = quarternionToEuler(pos->pose.pose.orientation);
	}
	
	void checkInputEnabled()
	{
		// if 'L1' is pressed on the controller, toggle reading controller values
		// TODO: Clean this mess up
		if (!this->l1 && this->l1_btn_state == button_up)
		{
			this->l1 = button_up;
		}
		else if (!this->l1 && this->l1_btn_state == button_down)
		{
			this->l1_btn_state = button_rising;
		}
		else if (this->l1 && this->l1_btn_state == button_up)
		{
			this->l1_btn_state = button_falling;
			this->inputEnabled = !this->inputEnabled;
			switch(this->inputEnabled)
			{
			case true:
				ROS_INFO("JoystickPlanner INFO: input is now enabled.");
				break;
			case false:
				ROS_INFO("JoystickPlanner INFO: input is now disabled.");
				break;
			}
		}
		else if (this->l1 && this->l1_btn_state == button_down)
		{
			this->l1_btn_state = button_down;
		}
		else if (!this->l1 && this->l1_btn_state == button_rising)
		{
			this->l1_btn_state = button_up;
		}
		else if (this->l1 && this->l1_btn_state == button_falling)
		{
			this->l1_btn_state = button_down;
		}
		
		if (this->xButton)
		{
			pc->cancelGoal();
		}
	}

    void filterData()
    {
        
		// Scale left stick input
        this->filt_leftStickX = this->scale_linear * this->leftStickX;
        this->filt_leftStickY = this->scale_linear * this->leftStickY;

        // Scale right stick input
        this->filt_rightStickX = this->scale_angular * this->rightStickX;
		this->filt_rightStickY = this->scale_angular * this->rightStickY;
    }
	
	void constructPath()
	{	
		// Scale raw controller input
		filterData();
		
		// Assign delta time value
		double tot_time = 0.02;
		double current_time = 0.0;
		
		// Create an std:vector for holding path positions
		std::vector<geometry_msgs::PoseStamped> poses(100);
		
		// Get the robot's orientation about the z-axis
		double current_y = this->robot_y_pos;
		double current_x = this->robot_x_pos;
		double current_joy_r_x = this->filt_rightStickX;
		double current_joy_l_x = this->filt_leftStickX;
		double current_joy_l_y = this->filt_leftStickY;
		double current_yaw = this->robot_orientation.yaw;
		if (current_yaw < 0){ current_yaw = 2 * M_PI - (current_yaw * -1); }
		current_yaw += current_joy_l_x * (M_PI / 2);
		
		// Send requested speed to the SpeedController
		std_msgs::Float32 speed_mod;
		speed_mod.data = this->calculateSpeed(current_joy_l_y);
		this->speed_modifier_pub.publish(speed_mod);
		
		if (current_joy_l_y != 0)
		{
			// populate the array
			for (int i = 0; i < 100; i++){
				geometry_msgs::PoseStamped p;
				p.header.stamp = ros::Time::now();
				p.header.frame_id = "odom";
				// calculate next x and y position values
				p.pose.position.x = current_x + (cos(current_yaw + current_joy_r_x * current_time) * current_time);
				p.pose.position.y = current_y + (sin(current_yaw + current_joy_r_x * current_time) * current_time);
				p.pose.position.z = 0;
				p.pose.orientation.x = 0.0;
				p.pose.orientation.y = 0.0;
				p.pose.orientation.z = 0.0;
				p.pose.orientation.w = 1.0;
				poses[i] = p;
				current_time += tot_time;
			}
			
			// create path object
			nav_msgs::Path _custom_path;
			_custom_path.header.frame_id = this->robot_base_frame;
			_custom_path.header.stamp = ros::Time::now();
			_custom_path.poses = poses;
			
			// create ActionGoal for 'move_base_flex'
			mbf_msgs::ExePathGoal customPath;
			customPath.controller = this->local_planner;
			customPath.path = _custom_path;
			
			// relay created ActionGoal to 'move_base_flex'
			pc->sendGoal(customPath);
			
			// pc.waitForResults(); // uncomment for result waiting
			}
		
	}

    void mapOutput(const sensor_msgs::Joy::ConstPtr &joy) 
	{
    	if (this->controller_mapping == "DS3")
    	{
		// DS3 mapping for buttons (using 'xboxdrv')
			this->xButton = joy->buttons[0];
			this->l1 = joy->buttons[4];
			this->leftStickX = joy->axes[0];
			this->leftStickY = joy->axes[1];
			this->rightStickX = joy->axes[2];
			this->rightStickY = joy->axes[3];
    	}
    	else
    	{
    		// DS4 mapping for buttons (using 'ds4drv')
			this->xButton = joy->buttons[1];
			this->l1 = joy->buttons[4];
			this->leftStickX = joy->axes[0];
			this->leftStickY = joy->axes[1];
			this->rightStickX = joy->axes[2];
			this->rightStickY = joy->axes[5];
    	}
	}
	Euler quarternionToEuler(geometry_msgs::Quaternion q)
	{
		Euler angles;
		angles.roll = atan2((2 * (q.w * q.x + q.y * q.z)), (1 - 2 * (pow(q.x, 2) + pow(q.y, 2))));
		angles.pitch = asin(2 * (q.w * q.y - q.z * q.x));
		angles.yaw = (atan2((2 * (q.w * q.z + q.x * q.y)), (1 - 2 * (pow(q.y, 2) + pow(q.z, 2)))));
		return angles;
	}
	
	double calculateSpeed(double joy_pos_l)
	{
		return 0.35 * joy_pos_l + 0.15;
	}
	
	void run()
	{
		this->checkInputEnabled();
		if (this->inputEnabled)
		{
			this->constructPath();
		}
		std_msgs::Bool msg;
		msg.data = this->inputEnabled;
		this->input_status_pub.publish(msg);
	}
};

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "JoystickPlanner");
    ros::NodeHandle n;

    // Create planner object
    JoystickPlanner *planner = new JoystickPlanner(n);
	
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        planner->run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete planner;
	return 0;
}
