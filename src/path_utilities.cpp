#include <iostream>
#include "rc_car_2dnav/path_utilities.h"

Trajectory::Trajectory(MPC &mpc_)
{
    // mpc = MPC();
    mpc = mpc_;

    traj_sub = nh.subscribe<nav_msgs::Path>("/trajectory", 10, &Trajectory::trajCallback, this);
    traj_pub = nh.advertise<nav_msgs::Path>("/trajectory_local", 1);
    closestPointMarker_pub = nh.advertise<visualization_msgs::Marker>("/closestPoint", 10);
    closestPathMarker_pub = nh.advertise<visualization_msgs::MarkerArray>("/closestPath", 10);
    predictedPathMarker_pub = nh.advertise<visualization_msgs::MarkerArray>("/predictedPath", 10);
    cmd_pub = nh.advertise<self_driving_rc_car::RcCarTeleop>("/rc_car/mpc_cmd", 1);

    amclpose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &Trajectory::carPoseCallback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Trajectory::odomCallback, this);

    last_steer_cmd = 0;
    last_throttle_cmd = 0;
}

void Trajectory::trajCallback(const nav_msgs::Path::ConstPtr &trajectoryMsg)
{
    // Get the global trajectory and set
    if (!globalPath.poses.size() > 0)
    // if(trajectoryMsg != nullptr)
    {
        ROS_INFO("Retrieving global trajectory...");
        globalPath = *trajectoryMsg;
        ROS_INFO("Retrieved path: %d points ...", globalPath.poses.size());
    }
}

void Trajectory::trajectoryPublish()
{
    traj_pub.publish(trajNormalized);
}

void Trajectory::setClosestPointMarker(double pos_x, double pos_y)
{
    closestPointMarker.header.frame_id = "map";
    closestPointMarker.header.stamp = ros::Time();
    closestPointMarker.type = visualization_msgs::Marker::SPHERE;
    closestPointMarker.action = visualization_msgs::Marker::ADD;
    closestPointMarker.pose.position.x = pos_x;
    closestPointMarker.pose.position.y = pos_y;
    closestPointMarker.pose.orientation.x = 0.0;
    closestPointMarker.pose.orientation.y = 0.0;
    closestPointMarker.pose.orientation.z = 0.0;
    closestPointMarker.pose.orientation.w = 1.0;
    closestPointMarker.scale.x = 0.1;
    closestPointMarker.scale.y = 0.1;
    closestPointMarker.scale.z = 0.1;
    closestPointMarker.color.a = 1.0; // Don't forget to set the alpha!
    closestPointMarker.color.r = 0.0;
    closestPointMarker.color.g = 1.0;
    closestPointMarker.color.b = 0.0;
}

void Trajectory::closestPointMarkerPublish()
{
    closestPointMarker_pub.publish(closestPointMarker);
}

void Trajectory::pathMarker(visualization_msgs::MarkerArray &markerArray, std::string frame_, const std::vector<double> pos_x, const std::vector<double> pos_y, tf::Quaternion q_, double r_, double g_, double b_, double a_)
{
    for (int id = 0; id < pos_x.size(); id++)
    {
        markerArray.markers[id].header.frame_id = frame_;
        markerArray.markers[id].header.stamp = ros::Time();
        markerArray.markers[id].type = visualization_msgs::Marker::CUBE;
        markerArray.markers[id].action = visualization_msgs::Marker::ADD;
        markerArray.markers[id].id = id;
        markerArray.markers[id].pose.position.x = pos_x[id];
        markerArray.markers[id].pose.position.y = pos_y[id];
        markerArray.markers[id].pose.orientation.x = q_[0];
        markerArray.markers[id].pose.orientation.y = q_[1];
        markerArray.markers[id].pose.orientation.z = q_[2];
        markerArray.markers[id].pose.orientation.w = q_[3];
        markerArray.markers[id].scale.x = 0.1;
        markerArray.markers[id].scale.y = 0.1;
        markerArray.markers[id].scale.z = 0.1;
        markerArray.markers[id].color.a = a_; // Don't forget to set the alpha!
        markerArray.markers[id].color.r = r_ - (0.05 * id);
        markerArray.markers[id].color.g = g_ - (0.05 * id);
        markerArray.markers[id].color.b = b_ - (0.05 * id);
    }
}

void Trajectory::pathMarkerPublish()
{
    closestPathMarker_pub.publish(closestPathMarker);
    predictedPathMarker_pub.publish(predictedPathMarker);
}

void Trajectory::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    velocity = odomMsg->twist.twist.linear.x;
}

void Trajectory::cmdPublish()
{
    cmd_pub.publish(mpc_msg);
}

void Trajectory::carPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclPoseMsg)
{
    poseCar = amclPoseMsg->pose.pose;
    // ROS_INFO("********** \n CAR: x: %.2f y: %.2f w: %.2f", poseCar.position.x, poseCar.position.y, poseCar.orientation.w);
    // Car Orientation
    tf::Quaternion q_car(
        poseCar.orientation.x,
        poseCar.orientation.y,
        poseCar.orientation.z,
        poseCar.orientation.w);
    q_car_global = q_car;

    tf::Matrix3x3 m_car(q_car);
    double car_roll, car_pitch, car_yaw;
    m_car.getRPY(car_roll, car_pitch, car_yaw);

    rpyCar[0] = car_roll;
    rpyCar[1] = car_pitch;
    rpyCar[2] = car_yaw;

    ROS_INFO("CAR: x: %.2f y: %.2f rpyCar[2]: %.2f", poseCar.position.x, poseCar.position.y, rpyCar[2] * 180 / M_PI);

    // ROS_INFO("rpy[0] %.2f rpy[1] %.2f rpy[2] %.2f",rpyCar[0],rpyCar[1],rpyCar[2]);
    coeffsLocalSet = false;
    nav_msgs::Path cleanedPath;
    if (globalPath.poses.size() > 0)
    {
        // cleanedPath = cleanPath(globalPath);
        // closestPathPoint = getClosestPathPoint(cleanedPath);
        // coeffsOfCurrentCarPose(closestPathPoint);

        closestPathPoint = getClosestPathPoint(globalPath);
        coeffsOfCurrentCarPose(closestPathPoint);
    }

    if (coeffsLocalSet)
    {
        double poly_inc = 0.1;
        int num_points = 10;
        closestPathMarker.markers.resize(num_points);
        ROS_INFO("Point %d-> a %.2f b %.2f c %.2f d %.2f", closestPathPoint, localCoeffs[0], localCoeffs[1], localCoeffs[2], localCoeffs[3]);
        std::vector<double> x_vals;
        std::vector<double> y_vals;

        float closestPoints_x = globalPath.poses[closestPathPoint].pose.position.x;
        float closestPoints_y = globalPath.poses[closestPathPoint].pose.position.y;

        // ROS_INFO("closestPoints_x: %.2f closestPoints_y: %.2f", closestPoints_x,closestPoints_y);

        // closestPathPoint Orientation
        tf::Quaternion q_closestPathPoint(
            globalPath.poses[closestPathPoint].pose.orientation.x,
            globalPath.poses[closestPathPoint].pose.orientation.y,
            globalPath.poses[closestPathPoint].pose.orientation.z,
            globalPath.poses[closestPathPoint].pose.orientation.w);

        // tf::Matrix3x3 m_closestPathPoint(q_closestPathPoint);
        // double point_roll,point_pitch,point_yaw;
        // m_closestPathPoint.getRPY(point_roll,point_pitch,point_yaw);

        for (int i = 1; i < num_points; i++)
        {
            float x_val, y_val;
            x_val = poly_inc * i;
            y_val = mpc.polyeval(localCoeffs, x_val);
            ROS_INFO("x_val: %.4f y_val: %.4f", i, x_val, i, y_val);
            // ROS_INFO("x_vals[%d]: %.8f y_vals[%d]: %.2f", i, x_vals[i], i, y_vals[i]);
            x_vals.push_back(x_val);
            y_vals.push_back(y_val);
        }
        // pathMarker(closestPathMarker, poly_inc*i,polyeval(localCoeffs,poly_inc*i), i);
        pathMarker(closestPathMarker, "base_link", x_vals, y_vals, q_closestPathPoint, 1.0, 1.0, 0.0);
    }

    // calculateCoeffs();
}

void Trajectory::calculateCoeffs()
{
    if (closestPathPoint >= 0)
    {
        coeffsOfCurrentCarPose(closestPathPoint);
    }
    if (coeffsLocalSet)
    {
        double poly_inc = 0.1;
        int num_points = 10;
        closestPathMarker.markers.resize(num_points);
        // ROS_INFO("Point %d-> a %.2f b %.2f c %.2f d %.2f",closestPathPoint, localCoeffs[0],localCoeffs[1],localCoeffs[2],localCoeffs[3]);
        std::vector<double> x_vals;
        std::vector<double> y_vals;

        float closestPoints_x = globalPath.poses[closestPathPoint].pose.position.x;
        float closestPoints_y = globalPath.poses[closestPathPoint].pose.position.y;

        // ROS_INFO("closestPoints_x: %.2f closestPoints_y: %.2f", closestPoints_x,closestPoints_y);

        // closestPathPoint Orientation
        tf::Quaternion q_closestPathPoint(
            globalPath.poses[closestPathPoint].pose.orientation.x,
            globalPath.poses[closestPathPoint].pose.orientation.y,
            globalPath.poses[closestPathPoint].pose.orientation.z,
            globalPath.poses[closestPathPoint].pose.orientation.w);

        // tf::Matrix3x3 m_closestPathPoint(q_closestPathPoint);
        // double point_roll,point_pitch,point_yaw;
        // m_closestPathPoint.getRPY(point_roll,point_pitch,point_yaw);

        for (int i = 0; i < num_points; i++)
        {
            x_vals.push_back(poly_inc * i);
            y_vals.push_back(mpc.polyeval(localCoeffs, poly_inc * i));
            // ROS_INFO("x_vals[%d]: %.2f y_vals[%d]: %.2f", i, x_vals[i], i, y_vals[i]);
        }
        // pathMarker(closestPathMarker, poly_inc*i,polyeval(localCoeffs,poly_inc*i), i);
        pathMarker(closestPathMarker, "base_link", x_vals, y_vals, q_closestPathPoint, 1.0, 1.0, 0.0);
    }
}

std::vector<double> Trajectory::calculate_latency_state(double car_x, double car_y, double car_psi, double v, double steer_value, double throttle_value, Eigen::VectorXd coeffs, double latency)
{
    // ROS_INFO("Calculating latency...");
    double dt = latency / 1000; //milisecond to second

    car_x += v * cos(car_psi) * dt;
    car_psi -= steer_value / Lf * dt;
    v += throttle_value * dt;
    double epsi = car_psi - atan(coeffs[1] + 2 * car_x * coeffs[2] + 3 * coeffs[3] * pow(car_x, 2));
    // ROS_INFO("epsi: %.4f car_psi: %.4f calc: %.4f", epsi, car_psi, atan(coeffs[1] + 2 * car_x * coeffs[2] + 3 * coeffs[3] * pow(car_x, 2)));

    double cte = mpc.polyeval(coeffs, 0) + v * sin(epsi) * dt;

    // ROS_INFO("cte: %.4f curr: %.4f v*sin(%.4f): %.4f", cte, polyeval(coeffs, 0), epsi * 180 / M_PI, v * sin(epsi) * dt);
    // ROS_INFO("Completed Calculating latency...");

    std::vector<double> result{v, cte, epsi};
    // return v, cte, epsi;
    return result;
}

void Trajectory::predictControl()
{
    if (coeffsLocalSet)
    {
        // ROS_INFO("***********");
        // double epsi = 0.0;
        // double cte = 0.0;
        double epsi_, cte_;

        // epsi = rpyCar[2] -  atan(localCoeffs[1] + 2 * poseCar.position.x * localCoeffs[2] + 3 * localCoeffs[3] * pow(poseCar.position.x, 2));
        // cte = polyeval(localCoeffs, 0) + velocity * sin(epsi);

        double velocity2 = 0; // DUmmy value. Just ignore velocity change since we dont know the effect of throtlle on acceleration

        // velocity = 0.5;

        std::vector<double> result;
        result = calculate_latency_state(poseCar.position.x, poseCar.position.y, rpyCar[2], velocity, last_steer_cmd, last_throttle_cmd, localCoeffs, 100);
        // velocity2, cte_, epsi_ = calculate_latency_state(poseCar.position.x, poseCar.position.y, rpyCar[2], velocity, last_steer_cmd, last_throttle_cmd, localCoeffs, 100);

        // ROS_INFO("Getting latency...");
        // ROS_INFO("vel: %.4f cte: %.4f epsi: %.4f", result[0], result[1], result[2]);

        Eigen::VectorXd state(6);
        state << 0, 0, 0, result[0], result[1], result[2];

        auto vars = mpc.Solve(state, localCoeffs);

        last_steer_cmd = vars[0] * 35 / 180 * M_PI; //rad
        last_throttle_cmd = vars[1];
        // ROS_INFO("steer: %.2f throttle: %.2f", vars[0], vars[1]);

        mpc_msg.forward = vars[1];
        mpc_msg.servo = vars[0];

        std::vector<double> mpc_x_vals;
        std::vector<double> mpc_y_vals;

        for (int i = 2; i < vars.size(); i++)
        {
            if (i % 2 == 0)
            {
                mpc_x_vals.push_back(vars[i]);
                // ROS_INFO("mpc_x_vals: %.2f", vars[i]);
            }
            else
            {
                mpc_y_vals.push_back(vars[i]);
                // ROS_INFO("mpc_y_vals: %.2f", vars[i]);
            }
        }
        if (mpc_x_vals.size() > 0)
        {
            predictedPathMarker.markers.resize(mpc_x_vals.size());
            pathMarker(predictedPathMarker, "base_link", mpc_x_vals, mpc_y_vals, q_car_global, 0.0, 1.0, 0.0);
        }

        // ROS_INFO("x: %.4f y: %.4f cte: %.4f epsi: %.4f",poseCar.position.x, polyeval(localCoeffs, poseCar.position.x), cte ,epsi);
    }
}

int Trajectory::getClosestPathPoint(const nav_msgs::Path globalTrajectory)
{
    nav_msgs::Path closestPath = globalTrajectory;
    double shortest_dist = 1.0e9;
    double smallest_theta = 1.0e9;
    int closest_point = 0;

    // Find closest path point to the car and return a path starting from that point
    for (int i = 0; i < globalTrajectory.poses.size(); i++)
    {
        double dist_x = 0;
        double dist_y = 0;
        double new_dist = 0.0;
        double new_theta = 0.0;

        // dist_x = globalTrajectory.poses[i].pose.position.x - poseCar.position.x;
        // dist_y = globalTrajectory.poses[i].pose.position.y - poseCar.position.y;

        tf::Pose traj_pose;
        tf::poseMsgToTF(globalTrajectory.poses[i].pose, traj_pose);
        tf::Pose poseCarTf;
        tf::poseMsgToTF(poseCar, poseCarTf);

        tf::Pose diffTF = traj_pose.inverseTimes(poseCarTf);

        geometry_msgs::Transform diffMsg;
        tf::transformTFToMsg(diffTF, diffMsg);
        // ROS_INFO("translation: x %.2f y %.2f, rotation: z %.2f",diffMsg.translation.x,diffMsg.translation.y, tf::getYaw(diffMsg.rotation) * 180 / 3.1415);

        double dist_xy = sqrt(diffMsg.translation.x * diffMsg.translation.x + diffMsg.translation.y * diffMsg.translation.y);
        double theta_angle = fabs(tf::getYaw(diffMsg.rotation));

        if (dist_xy < shortest_dist)
        {
            if (fabs(theta_angle) < M_PI * 3.0 / 4.0)
            {
                shortest_dist = dist_xy;
                smallest_theta = theta_angle;
                closest_point = i;
            }
        }
    }
    // If closest point found get x,y positions and z rotation
    if (closest_point >= 0)
    {
        double pos_x = globalTrajectory.poses[closest_point].pose.position.x;
        double pos_y = globalTrajectory.poses[closest_point].pose.position.y;
        double rot_z = tf::getYaw(globalTrajectory.poses[closest_point].pose.orientation);
        // ROS_INFO("shortest dist %.2f @ %d x: %.2f y: %.2f z: %.2f",shortest_dist,closest_point, pos_x, pos_y,rot_z);
        // Show the closest point on RVIZ as a marker
        setClosestPointMarker(pos_x, pos_y);
    }

    return closest_point;
}

void Trajectory::coeffsOfCurrentCarPose(int point)
{
    std::vector<double> ptsx, ptsy;

    // Limit the number of trajectory points to be shifted
    u_int coeff_N = 6;

    if (globalPath.poses.size() != 0)
    {
        // ROS_INFO("poseCar x: %.2f y: %.2f",poseCar.position.x,poseCar.position.y);
        // closestPathMarker.markers.resize(coeff_N);
        // Shift Path Points
        for (int i = 0; i < coeff_N; i++)
        {
            /*
                 *  Standard Shifting Method
                 */
            // // Shift
            // double shift_x = globalPath.poses[point + i].pose.position.x - poseCar.position.x;
            // double shift_y = globalPath.poses[point + i].pose.position.y - poseCar.position.y;
            // // Rotate
            // ROS_INFO("rpy[2]: %.2f",rpyCar[2]*180/M_PI);
            // float rot_x = (shift_x * CppAD::cos(0 - rpyCar[2])) - (shift_y * CppAD::sin(0 - rpyCar[2]));
            // float rot_y = (shift_x * CppAD::sin(0 - rpyCar[2])) + (shift_y * CppAD::cos(0 - rpyCar[2]));
            // // Push
            // ptsx.push_back(rot_x);
            // ptsy.push_back(rot_y);

            /*
                 * Shifting with #include <tf/transform_datatypes.h> method
                 */
            tf::Pose tfPath;
            tf::poseMsgToTF(globalPath.poses[point + i].pose, tfPath);

            tf::Pose tfCar;
            tf::poseMsgToTF(poseCar, tfCar);

            tf::Pose shiftedPose;
            shiftedPose = tfCar.inverseTimes(tfPath);
            // shiftedPose = tfPath.inverseTimes(tfCar);
            // std::cout << "x: " << shiftedPose.getOrigin().getX() << " y: " << shiftedPose.getOrigin().getY() << std::endl;
            // std::cout << "z: " << shiftedPose.getRotation().getZ() << std::endl;

            ptsx.push_back(shiftedPose.getOrigin().getX());
            ptsy.push_back(shiftedPose.getOrigin().getY());
        }

        // ROS_INFO("trajNormalized.poses.size(): %d",shiftedPath.poses.size());
        double *ptrx = &ptsx[0];
        Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, coeff_N);
        double *ptry = &ptsy[0];
        Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, coeff_N);

        auto coeffs = mpc.polyfit(ptsx_transform, ptsy_transform, 3);

        // ROS_INFO("FIRST VARIABLES-> a: %.2f b: %.2f c: %.2f d: %.2f",coeffs[0],coeffs[1],coeffs[2],coeffs[3]);

        localCoeffs = coeffs;
        coeffsLocalSet = true;
    }
}
