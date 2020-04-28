#ifndef PATH_CONTROLLER_H_
#define PATH_CONTROLLER_H_

#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

class PathController{
private:
	ros::NodeHandle nh;

	geometry_msgs::PoseStamped currentPose;
	nav_msgs::Path savedPath;

	ros::Subscriber joy_sub;
	ros::Subscriber amcl_pose_sub;

	ros::Publisher path_pub;	

public:
	PathController();
	void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
	void joyCommandPublish();

	void AMCLposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &AMCLmsg);

	void savePath();
	void printPath();
	void publishPath();

	bool recordingPath = false;
    bool printingPath = false;
    bool publishingPath = false;
};

PathController::PathController(){
	// teleop_cmd_pub = nh.advertise<self_driving_rc_car::RcCarTeleop>("rc_car/teleop_cmd",1);
	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy",10, &PathController::joyCallback,this);
	amcl_pose_sub = nh.subscribe("amcl_pose",10, &PathController::AMCLposeCallback,this);

	path_pub = nh.advertise<nav_msgs::Path>("/rc_car/savedPath",10);
}


#endif