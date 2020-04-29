#include <iostream>
#include "ros/ros.h"
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include <ackermann_msgs/AckermannDriveStamped.h>


float rad2deg(float rad)
{
    return rad * 180 / M_PI;
}

float deg2rad(float deg)
{
    return deg * M_PI / 180;
}

class TEBpathFollower
{
    public:
        float lf = 0.22;
       

    private:
        ros::Time current_time, prev_time;
        ros::Duration delta_time; double dt;

        float v_cmd_x,v_cmd_y,rot_cmd_z;
        float v_odom_x,v_odom_y,rot_odom_z;

        ackermann_msgs::AckermannDriveStamped ackermann_msg;


    public:
        TEBpathFollower()
        {
            ROS_INFO_STREAM("Constructor called");
         }
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg)
        {
            current_time = ros::Time::now();
            delta_time = current_time - prev_time;
            dt = delta_time.toSec();

            v_cmd_x = cmd_vel_msg->linear.x;
            v_cmd_y = cmd_vel_msg->linear.y;
            rot_cmd_z = cmd_vel_msg->angular.z;

            // ROS_INFO("cmd_vel_cb-> v_cmd_x: %.2f",v_cmd_x);


            prev_time = current_time;
        }     
        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
        {
            v_odom_x = odom_msg->twist.twist.linear.x;
            v_odom_y = odom_msg->twist.twist.linear.x;
            rot_odom_z = odom_msg->twist.twist.angular.z;

            // ROS_INFO("odom_cb-> v_odom_x: %.2f",v_odom_x);
        }     
        
        float convert_trans_rot_vel_to_steering_angle(const float v,const float omega,const float wheelbase)
        {
            // ROS_INFO("conversion...");
            if(omega == 0 || v == 0){return 0;}

            float radius = v / omega;
            float steering_angle = atan2(wheelbase , radius); //rad

            return steering_angle;
        }

        ackermann_msgs::AckermannDriveStamped calculate_ackermann_msg()
        {            
            // Steering angle transformed from cmd_vel
            float steering_cmd = convert_trans_rot_vel_to_steering_angle(v_cmd_x,rot_cmd_z,lf);

            float v_x_diff = v_cmd_x - v_odom_x;
           
            ackermann_msg.header.stamp = ros::Time::now();
            ackermann_msg.drive.steering_angle = steering_cmd;
            ackermann_msg.drive.speed = v_x_diff;

            ROS_INFO("v_ack_x: %.2f, steer: %.2f lf: %.2f",ackermann_msg.drive.speed, ackermann_msg.drive.steering_angle, lf);

            return ackermann_msg;
        }

};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ackermann_publisher");

    ros::NodeHandle nh;

    TEBpathFollower tebPF;

    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &TEBpathFollower::cmd_vel_callback, &tebPF);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &TEBpathFollower::odom_callback, &tebPF);

    ros::Publisher teleop_cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/rc_car/ackermann_cmd",1);
    ros::Rate loop_rate(50);

    if (!nh.getParam ("/rc_car/lf", tebPF.lf)){ tebPF.lf = 0.22;}      

    while(ros::ok())
    {
        // teleop_cmd_pub.publish(tebPF.calculate_ackermann_msg());

        ros::spinOnce();  
        loop_rate.sleep();

    }
   
    return 0;
}