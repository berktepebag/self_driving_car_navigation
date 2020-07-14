#ifndef PATH_UTILITIES_H
#define PATH_UTILITIES_H

#include "rc_car_2dnav/MPC.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "self_driving_rc_car/RcCarTeleop.h"
#include <tf/transform_datatypes.h>

// https://stackoverflow.com/questions/2133250/x-does-not-name-a-type-error-in-c/2133260

class Trajectory
{
public:
    ros::NodeHandle nh;
    int closestPathPoint;

private:
    ros::Subscriber traj_sub;
    ros::Publisher traj_pub;
    ros::Publisher closestPointMarker_pub;
    ros::Publisher closestPathMarker_pub;
    ros::Publisher predictedPathMarker_pub;
    ros::Publisher cmd_pub;

    ros::Subscriber amclpose_sub;
    ros::Subscriber odom_sub;

    nav_msgs::Path globalPath;
    nav_msgs::Path trajNormalized;
    nav_msgs::Path trajCalculated;

    geometry_msgs::Pose poseCar;
    std::vector<double> rpyCar{0, 0, 0};

    Eigen::VectorXd localCoeffs;
    bool coeffsLocalSet = false;

    double velocity;

    MPC mpc;

    visualization_msgs::Marker closestPointMarker;
    visualization_msgs::MarkerArray closestPathMarker;
    visualization_msgs::MarkerArray predictedPathMarker;

    self_driving_rc_car::RcCarTeleop mpc_msg;

    tf::Quaternion q_car_global;

    double last_steer_cmd, last_throttle_cmd;

    const double Lf = 0.22; //TODO: make this global parameter

public:
    Trajectory(MPC &mpc_);

    void trajCallback(const nav_msgs::Path::ConstPtr &trajectoryMsg);

    void trajectoryPublish();

    void setClosestPointMarker(double pos_x, double pos_y);

    void closestPointMarkerPublish();

    void pathMarker(visualization_msgs::MarkerArray &markerArray, std::string frame_, const std::vector<double> pos_x, const std::vector<double> pos_y, tf::Quaternion q_, double r_ = 0.0, double g_ = 0.0, double b_ = 0.0, double a_ = 1.0);

    void pathMarkerPublish();

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

    void cmdPublish();

    void carPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclPoseMsg);

    void calculateCoeffs();

    std::vector<double> calculate_latency_state(double car_x, double car_y, double car_psi, double v, double steer_value, double throttle_value, Eigen::VectorXd coeffs, double latency);

    void predictControl();

    int getClosestPathPoint(const nav_msgs::Path globalTrajectory);

    void coeffsOfCurrentCarPose(int point);
};
#endif