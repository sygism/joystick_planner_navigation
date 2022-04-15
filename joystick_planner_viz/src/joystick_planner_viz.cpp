/*
 *  User interface for JoystickPlanner
 *  File: joystick_planner_viz.cpp
 *
 *  author: Markus Erik Sügis <markus.sugis@gmail.com>
 */

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

using namespace cv;

class JoystickPlannerViz{
	
	struct Euler
	{
		double roll, pitch, yaw;
	};

	private:
		ros::Subscriber robot_odometry_sub;
		ros::Subscriber input_enabled_sub;
		ros::Subscriber joy_pos_sub;
		
		// transport layer for opencv
		image_transport::Subscriber stream;
		
		// Std vector for holding path poses
		std::vector<cv::Point> global_path;
		// Euler object for storing robot's orientation
		Euler robot_orientation;
		double robot_x;
		double robot_y;
		// boolean for displaying whether input is enabled
	    bool input_enabled;
	    // default window name
	    std::string window_name = "JoystickPlanner UI";
	    
	    // Constants
	    cv::Vec3b c_green = {0, 255, 0};
	    
	    // Joy values
	    double l_stick_x = 0;
	    double r_stick_x = 0;
	    
	    // Variables for params
	    std::string image_feed_topic;
	    std::string odom_topic;
	    std::string input_enabled_topic;
	    std::string joy_topic;
	    bool show_input_enabled = false;
	    bool show_robot_orientation = false;
	    bool show_heading = false;
	
	protected:
	    cv_bridge::CvImagePtr img_ptr;
		
	public:
		
		JoystickPlannerViz(ros::NodeHandle &n)
		{
			ros::NodeHandle private_nh("~");
			image_transport::ImageTransport it(n);
			
			// Initialize params
			private_nh.param<std::string>("image_feed_topic", this->image_feed_topic, "camera/color/image_raw");
			private_nh.param<std::string>("joy_node", this->joy_topic, "/joy");
			private_nh.param<std::string>("odom_topic", this->odom_topic, "odom");
			private_nh.param<std::string>("input_enabled_topic", this->input_enabled_topic, "joystick_planner/input_status");
			private_nh.param<bool>("show_input_enabled", this->show_input_enabled, true);
			private_nh.param<bool>("show_robot_orientation", this->show_robot_orientation, true);
			
			
			// subscribe to 'global_path' topic to fetch current global path for visualization
			this->joy_pos_sub = n.subscribe<sensor_msgs::Joy>("/joy", 10, &JoystickPlannerViz::readJoyValues, this);
			// subscribe to '/odometry/filtered' topic where the robot's odometry is published to
			this->robot_odometry_sub = n.subscribe<nav_msgs::Odometry>(this->odom_topic, 10,  &JoystickPlannerViz::updateRobotPosition, this);
			this->input_enabled_sub = n.subscribe<std_msgs::Bool>(this->input_enabled_topic, 10, &JoystickPlannerViz::isInputEnabled, this);
			this->stream = it.subscribe(this->image_feed_topic, 30, &JoystickPlannerViz::transportFeed, this);
			
			// create named window for camera feed visualization
			cv::namedWindow(this->window_name, cv::WINDOW_NORMAL);
			// resize window to 1280x720 resolution
			cv::resizeWindow(this->window_name, 1280, 720);
			cvui::init(this->window_name);
		}
		
		void updateRobotPosition(const nav_msgs::Odometry::ConstPtr &pos)
		{
			// fetch the robot's orientation as a quaternion and convert it to Euler angles
			this->robot_orientation = quarternionToEuler(pos->pose.pose.orientation);
			this->robot_x = pos->pose.pose.position.x;
			this->robot_y = pos->pose.pose.position.y;
		}
		
		void isInputEnabled(const std_msgs::Bool::ConstPtr &enabled)
		{
			this->input_enabled = enabled->data;
		}
		
		void readJoyValues(const sensor_msgs::Joy::ConstPtr &joy){
			this->l_stick_x = joy->axes[0];
			this->r_stick_x = joy->axes[2] * 0.5;
		}
		
		std::vector<cv::Point> projectGlobalPath(){
			int x, y;
			std::vector<cv::Point> projection(20);
			int scalar = 0;
			int n_rows = this->img_ptr->image.rows;
			int n_cols = this->img_ptr->image.cols / 2;
			
			for (int i = 0; i < 20; i++){
				x = (int) (n_rows - (sin(this->l_stick_x * (M_PI / 2) + this->r_stick_x * scalar) * scalar));
				y = (int) (n_cols - (cos(this->l_stick_x * (M_PI / 2) + this->r_stick_x * scalar) * scalar));
				projection[i] = cv::Point(x, y);
				scalar += 15;
			}
			
			return projection;
		}
		
		void displayRPY(){
			std::string orientation_roll = "Roll: " + std::to_string(this->robot_orientation.roll) + " rad";
			std::string orientation_pitch = "Pitch: " + std::to_string(this->robot_orientation.pitch) + " rad";
			std::string orientation_yaw = "Yaw: " + std::to_string(this->robot_orientation.yaw) + " rad";
			cv::putText(this->img_ptr->image,
						orientation_roll,
						cv::Point(10, this->img_ptr->image.rows / 2),
						cv::FONT_HERSHEY_DUPLEX,
						1.0,
						CV_RGB(255, 255, 255),
						2);
			cv::putText(this->img_ptr->image,
						orientation_pitch,
						cv::Point(10, (this->img_ptr->image.rows / 2) + 30),
						cv::FONT_HERSHEY_DUPLEX,
						1.0,
						CV_RGB(255, 255, 255),
						2);
			cv::putText(this->img_ptr->image,
						orientation_yaw,
						cv::Point(10, (this->img_ptr->image.rows / 2) + 60),
						cv::FONT_HERSHEY_DUPLEX,
						1.0,
						CV_RGB(255, 255, 255),
						2);
		}
		
		void displayInputEnabled(){
			if (this->input_enabled){
				cv::putText(this->img_ptr->image,
					"Planner status: input is enabled!",
					cv::Point(10, 25),
					cv::FONT_HERSHEY_DUPLEX,
					1.0,
					CV_RGB(0, 255, 0),
					2);
			}
			else{
				cv::putText(this->img_ptr->image,
					"Planner status: input is disabled!",
					cv::Point(10, 25),
					cv::FONT_HERSHEY_DUPLEX,
					1.0,
					CV_RGB(255, 0, 0),
					2);
			}
		}
		
		Euler quarternionToEuler(geometry_msgs::Quaternion q)
		{
			Euler angles;
			angles.roll = atan2((2 * (q.w * q.x + q.y * q.z)), (1 - 2 * (pow(q.x, 2) + pow(q.y, 2))));
			angles.pitch = asin(2 * (q.w * q.y - q.z * q.x));
			angles.yaw = atan2((2 * (q.w * q.z + q.x * q.y)), (1 - 2 * (pow(q.y, 2) + pow(q.z, 2))));
			return angles;
		}
		
		void transportFeed(const sensor_msgs::ImageConstPtr &frame){
			
			try
			{
				this->img_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception &e)
			{
				ROS_ERROR("JoystickPlannerViz ERROR: error whilst converting raw feed to cv::mat (cv_bridge). Exception: %s", e.what());
				return;
			}
		}
		
		void displayFeed()
		{
			if (img_ptr)
			{
				if (this->show_heading){
					if (this->input_enabled){
						std::vector<cv::Point> path = this->projectGlobalPath();
						for (int i = 0; i < 20; i++){
							cv::circle(img_ptr->image, path[i], 5, c_green, CV_FILLED);
						}
					}
				}
				if (this->show_robot_orientation){
					displayRPY();
				}
				if (this->show_input_enabled){
					displayInputEnabled();
				}
			
				
				cvui::checkbox(this->img_ptr->image, 1600, 600, "Display robot orientation", &this->show_robot_orientation, 0xFFFFFF, 0.5);
				cvui::checkbox(this->img_ptr->image, 1600, 650, "Display set heading", &this->show_heading, 0xFFFFFF, 0.5);
				cvui::checkbox(this->img_ptr->image, 1600, 700, "Display planner status", &this->show_input_enabled, 0xFFFFFF, 0.5);
				cv::imshow(this->window_name, this->img_ptr->image);
			}
			cv::waitKey(1);
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "JoystickPlannerViz");
    ros::NodeHandle n;

    // Create visualizer object
    JoystickPlannerViz *visualizer = new JoystickPlannerViz(n);
    
    ROS_INFO("JoystickPlannerViz INFO: Started visualizer node.");
	
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
    	visualizer->displayFeed();
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete visualizer;
	return 0;
}