#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>
#include <assert.h>

//rosrun hector_trajectory_server hector_trajectory_server 

class Trajectory{
    private:
      
        ros::Subscriber traj_sub;
        ros::Publisher traj_pub;

        rosbag::Bag bag;
    public:
        ros::NodeHandle nh;

        bool savingTrajectory;
        bool loadNpubTrajectory;

        std::string bag_name = "";
        std::string folder_path = "";
        std::string full_path = "";
                
    public:

        Trajectory()
        {            
            traj_sub = nh.subscribe<nav_msgs::Path>("/trajectory",10, &Trajectory::trajCallback, this); 
            traj_pub = nh.advertise<nav_msgs::Path>("/trajectory",1);            
        }
        ~Trajectory()
        {
            ROS_INFO("trajectory saver bye bye");
        }

        void trajCallback(const nav_msgs::Path::ConstPtr &trajectoryMsg)
        {
            int size = trajectoryMsg->poses.size();
            // ROS_INFO("size: %d",size);

            if(savingTrajectory & size > 2)
            {
                ROS_INFO("Opening new bag...");
                
                bag.open(full_path, rosbag::bagmode::Write);
                bag.write("trajectory", trajectoryMsg->header.stamp, trajectoryMsg);
                savingTrajectory = false;
                bag.close();
                ROS_INFO("Completed writing bag");
            }
        }

        void trajectoryPublish()
        {
            bag.open(full_path, rosbag::bagmode::Read);
            
            std::vector<std::string> topics = {"trajectory"};
            // topics.push_back(std::string("trajectory"));
            
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            // ROS_INFO("reading topics...");
            foreach(rosbag::MessageInstance const m, view)
            {
                // ROS_INFO("reading messages...");
                nav_msgs::Path::ConstPtr path = m.instantiate<nav_msgs::Path>();
                if(path != NULL)
                {
                    // ROS_INFO_STREAM(path);
                    traj_pub.publish(path);
                }
            }
            bag.close();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_utilties");
    Trajectory tjs;

    ros::NodeHandle nh_private("~");

    // Load parameters
    if (!nh_private.getParam ("save", tjs.savingTrajectory)){ tjs.savingTrajectory = false;}      
    if (!nh_private.getParam ("load", tjs.loadNpubTrajectory)){ tjs.loadNpubTrajectory = false;}      
    if (!nh_private.getParam("bag_name", tjs.bag_name)){tjs.bag_name = "trajectory.bag";};
    if (!nh_private.getParam("path", tjs.folder_path)){tjs.folder_path = "/home/berk/catkin_ws/src/SDRC/rc_car_2dnav/bags/";};

    //Cannot load and save in the same time. Check if parameters are set to same.
    assert(tjs.savingTrajectory != tjs.loadNpubTrajectory);
    
    // Set bag name and path and check
    tjs.full_path = tjs.folder_path + tjs.bag_name;
    assert(!tjs.full_path.empty());

    ROS_INFO("Bag path: %s", tjs.folder_path.c_str());
    ROS_INFO("Bag name: %s",tjs.bag_name.c_str());
    // Loading or saving bag
    if(tjs.loadNpubTrajectory) ROS_INFO("Loading bag and publishing to /trajectory.");
    if(tjs.savingTrajectory) ROS_INFO("Saving trajectory to bag.");

    ros::Rate rate(1);
    while(ros::ok())
    {
        // ROS_INFO_STREAM(tjs.loadNpubTrajectory);
        if(tjs.loadNpubTrajectory)
        {
            // ROS_INFO("loading bag...");
            tjs.trajectoryPublish();
        }
        ros::spinOnce();

        rate.sleep();
    }
    return 0;
}