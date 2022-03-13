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
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;

class JoystickPlannerViz{
	
	struct Euler
	{
		double roll, pitch, yaw;
	};

	private:
		ros::Subscriber global_path_sub;
		ros::Subscriber local_path_sub;
		ros::Subscriber robot_odometry_sub;
		ros::Subscriber input_enabled_sub;
		
		// transport layer for opencv
		image_transport::Subscriber stream;
		
		// Std vector for holding path poses
		std::vector<cv::Point> global_path;
		std::vector<cv::Point> local_path;
		// Euler object for storing robot's orientation
		Euler robot_orientation;
		double robot_x;
		double robot_y;
		// boolean for displaying whether input is enabled
	    bool input_enabled;
	    // default window name
	    std::string window_name = "JoystickPlanner UI";
	    
	    // Variables for projection calculations
	    int window_midpoint;
	    
	    // Constants
	    cv::Vec3b c_green = {0, 255, 0};
	    
	    // Variables for params
	    std::string image_feed_topic;
	    std::string global_path_topic;
	    std::string local_path_topic;
	    std::string odom_topic;
	    std::string input_enabled_topic;
	    int window_width;
	    int window_height;
	    bool show_global_path;
	    bool show_local_path;
	    bool show_input_enabled;
	    bool show_robot_orientation;
	
	protected:
	    cv_bridge::CvImagePtr img_ptr;
		
	public:
		
		JoystickPlannerViz(ros::NodeHandle &n)
		{
			ros::NodeHandle private_nh("~");
			image_transport::ImageTransport it(n);
			
			// Initialize params
			private_nh.param<std::string>("image_feed_topic", this->image_feed_topic, "camera/color/image_raw");
			private_nh.param<std::string>("global_path_topic", this->global_path_topic, "/move_base_flex/teb_local_planner/TebLocalPlannerROS/global_plan");
			private_nh.param<std::string>("local_path_topic", this->local_path_topic, "local_path");
			private_nh.param<std::string>("odom_topic", this->odom_topic, "odom");
			private_nh.param<std::string>("input_enabled_topic", this->input_enabled_topic, "joystick_planner/input_enabled");
			private_nh.param<int>("window_width", this->window_width, 1280);
			private_nh.param<int>("window_height", this->window_height, 720);
			private_nh.param<bool>("show_global_path", this->show_global_path, true);
			private_nh.param<bool>("show_local_path", this->show_local_path, true);
			private_nh.param<bool>("show_input_enabled", this->show_input_enabled, true);
			private_nh.param<bool>("show_robot_orientation", this->show_robot_orientation, true);
			
			this->window_midpoint = (int)(this->window_width / 2);
			
			// subscribe to 'global_path' topic to fetch current global path for visualization
			this->global_path_sub = n.subscribe<nav_msgs::Path>(this->global_path_topic, 10, &JoystickPlannerViz::updateCurrentGlobalPath, this);
			// subscribe to 'local_path' topic to fetch current local path for visualization
			this->local_path_sub = n.subscribe<nav_msgs::Path>(this->local_path_topic, 10, &JoystickPlannerViz::updateCurrentLocalPath, this);
			// subscribe to '/odometry/filtered' topic where the robot's odometry is published to
			this->robot_odometry_sub = n.subscribe<nav_msgs::Odometry>(this->odom_topic, 10,  &JoystickPlannerViz::updateRobotPosition, this);
			this->input_enabled_sub = n.subscribe<std_msgs::Bool>(this->input_enabled_topic, 10, &JoystickPlannerViz::isInputEnabled, this);
			this->stream = it.subscribe(this->image_feed_topic, 30, &JoystickPlannerViz::transportFeed, this);
			
			// create named window for camera feed visualization
			cv::namedWindow(this->window_name, cv::WINDOW_NORMAL);
			// resize window to 1280x720 resolution
			cv::resizeWindow(this->window_name, this->window_width, this->window_height);
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
		
		void updateCurrentGlobalPath(const nav_msgs::Path::ConstPtr &path)
		{
			this->global_path = projectGlobalPath(path->poses);
		}
		
		void updateCurrentLocalPath(const nav_msgs::Path::ConstPtr &path)
		{
			// this->local_path = projectGlobalPath(path->poses);
		}
		
		
		std::vector<cv::Point> projectGlobalPath(std::vector<geometry_msgs::PoseStamped> path){
			std::vector<cv::Point> projection(20);
			
			int norm_x = path[0].pose.position.x;
			int norm_y = path[0].pose.position.y;
			int transform_x;
			int transform_y;
			
			for (int i = 0; i < 100; i+=5){
				transform_x = cos(M_PI / 2) * (path[i].pose.position.x - norm_x) -
							  sin(M_PI / 2) * (path[i].pose.position.y - norm_y);
				transform_y = cos(M_PI / 2) * (path[i].pose.position.y - norm_y) +
							  sin(M_PI / 2) * (path[i].pose.position.x - norm_x);
				projection[i] = cv::Point(transform_x, transform_y);
			}
			return projection;
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
				if ((this->global_path).size() != 0){
					for (int i = 0; i < 20; i++){
						img_ptr->image.at<Vec3b>(global_path[i]) = c_green;
						ROS_INFO("draw point at: %d, %d", global_path[i].x, global_path[i].y);
					}
				}
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