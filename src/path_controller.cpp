#include "rc_car_2dnav/path_controller.h"


void PathController::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    //LT:joy->axes[2]
    //RT:joy->axes[5]
    //Left Stick Horizontal: joy->axes[0]
    //Left Stick Vertical: joy->axes[1]
    //Right Stick Horizontal: joy->axes[3]
    //Right Stick Vertical: joy->axes[4]
    /*
    A: 0
    B: 1
    X: 2
    Y: 3
    LB: 4
    RB: 5
    Back: 6
    Start: 7
    */

    if (joy->buttons[0])
    {
        recordingPath = !recordingPath;
        ROS_INFO_STREAM("recording Path: " << recordingPath);
    }
    if (joy->buttons[1])
    {
        printingPath = !printingPath;
        ROS_INFO_STREAM("printing Path: " << printingPath);    
    }
    if (joy->buttons[2])
    {
        publishingPath = !publishingPath;
        ROS_INFO_STREAM("publishing Path: " << publishingPath);    
    }
}

void PathController::AMCLposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &AMCLmsg)
{
    ROS_INFO("pose.x: %.2f pose.y: %.2f rot.w: %.2f", AMCLmsg->pose.pose.position.x,AMCLmsg->pose.pose.position.y,AMCLmsg->pose.pose.orientation.w);

    currentPose.header.stamp = ros::Time::now();
    currentPose.header.frame_id = AMCLmsg->header.frame_id;
    currentPose.pose.position.x = AMCLmsg->pose.pose.position.x;
    currentPose.pose.position.y = AMCLmsg->pose.pose.position.y;
    currentPose.pose.orientation.w = AMCLmsg->pose.pose.orientation.w;

    if(recordingPath)
    {
        savePath();
    }
}

void PathController::savePath()
{
    ROS_INFO("Saving pose to path...");
    savedPath.poses.push_back(currentPose);
}

void PathController::printPath()
{
    int pathSize = savedPath.poses.size();
    for(int i=0; i < pathSize; i++)
    {
     ROS_INFO("pos_x %.2f pos_y %.2f rot_w: %.2f",savedPath.poses[i].pose.position.x, savedPath.poses[i].pose.position.y, savedPath.poses[i].pose.orientation.w);
    }
    printingPath = false;
}

void PathController::publishPath()
{
    path_pub.publish(savedPath);
}

int main(int argc, char** argv)
{
    ROS_INFO("Hello");
    ros::init(argc,argv,"path_controller");

    PathController pathCtrl;
    
    while(ros::ok)
    {
        // ROS_INFO("hello");
       
        if(pathCtrl.printingPath){pathCtrl.printPath();}
        if(pathCtrl.publishingPath){pathCtrl.publishPath();}

        ros::spinOnce();
    }
    return 0;
}