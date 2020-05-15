#include <iostream>

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/QR"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>
// using CppAD::AD;

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 0.5;

class MPC{
    public:
        MPC();
        virtual ~MPC();

        std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

size_t N_timesteps = 4; // timestep length
double dt = 0.2; // timestep duration

const double Lf = 0.22; //TODO: make this global parameter

size_t x_start = 0;
size_t y_start = x_start + N_timesteps;
size_t psi_start = y_start + N_timesteps;
size_t v_start = psi_start + N_timesteps;
size_t cte_start = v_start + N_timesteps;
size_t epsi_start = cte_start + N_timesteps;
size_t delta_start = epsi_start + N_timesteps;
size_t a_start = delta_start + N_timesteps - 1;

class FG_eval
{
    public:
        Eigen::VectorXd coeffs;
        FG_eval(Eigen::VectorXd coeffs) {this->coeffs = coeffs;}

        typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
        void operator()(ADvector &fg, const ADvector &vars)
        {
            // Initial Constraints
            fg[1+x_start] = vars[x_start];
            fg[1+y_start] = vars[y_start];
            fg[1+psi_start] = vars[psi_start];
            fg[1+v_start] = vars[v_start];
            fg[1+cte_start] = vars[cte_start];
            fg[1+epsi_start] = vars[epsi_start];

            // Rest of the constraints
            for(int i=0; i<N_timesteps - 1; i++)
            {
                CppAD::AD<double> x1 = vars[x_start+i+1];
                CppAD::AD<double> y1 = vars[y_start+i+1];
                CppAD::AD<double> psi1 = vars[psi_start+i+1];
                CppAD::AD<double> v1 = vars[v_start+i+1];
                CppAD::AD<double> cte1 = vars[cte_start+i+1];
                CppAD::AD<double> epsi1 = vars[epsi_start+i+1];
                
                CppAD::AD<double> x0 = vars[x_start+i];
                CppAD::AD<double> y0 = vars[y_start+i];
                CppAD::AD<double> psi0 = vars[psi_start+i];
                CppAD::AD<double> v0 = vars[v_start+i];
                CppAD::AD<double> cte0 = vars[cte_start+i];
                CppAD::AD<double> epsi0 = vars[epsi_start+i];

                CppAD::AD<double> delta0 = vars[delta_start+i];
                CppAD::AD<double> a0 = vars[a_start+i];

                // Reference Path
                CppAD::AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
                CppAD::AD<double> psides0 = CppAD::atan(coeffs[1]*2*x0*coeffs[2]+3*x0*x0*coeffs[3]);

                fg[2+i+x_start] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
                fg[2+i+y_start] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
                fg[2+i+psi_start] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
                fg[2+i+v_start] = v1 - (v0 + a0 * dt);
                fg[2+i+cte_start] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
                fg[2+i+epsi_start] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
            }

            fg[0] = 0;
            for(int i=0; i < N_timesteps; i++)
            {
                fg[0] += 2000 * CppAD::pow(vars[cte_start+i],2); // CTE error
                fg[0] += 2000 * CppAD::pow(vars[epsi_start+i],2); // Psi error
                fg[0] += CppAD::pow(vars[v_start+i] - ref_v,2);
            }
        }
};

MPC::MPC() {};
MPC::~MPC() {};

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    bool ok=true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    size_t n_vars = 6 * N_timesteps + 2 * (N_timesteps - 1); // number of varibles
    size_t n_constraints = 6 * N_timesteps; // number of constraints

    Dvector vars(n_vars); // initial condition of variables
    for(int i=0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    // lower and upper bounds for varibles
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    for(int i=0; i < delta_start; i++)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }    
    for(int i=delta_start; i < a_start; i++)
    {
        vars_lowerbound[i] = -0.4363 * Lf;
        vars_upperbound[i] = 0.4363 * Lf;
    }    
    for(int i=a_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }    
     // lower and upper bounds for constraints
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for(int i=0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0.0;
        constraints_upperbound[i] = 0.0;
    }    
    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    FG_eval fg_eval(coeffs);

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String sb            yes\n";
    // maximum iterations
    options += "Integer max_iter     10\n";
    //approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
    //derivative tesing
    options += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius   0.\n";
    options += "Sparse  true        forward\n";
    // options += "Numeric max_cpu_time          0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution; // solution
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution); // solve the problem

    // std::cout<<"solution: "<<solution.x<< std::endl;

    //
    //check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    //
  
    std::vector<double> result;

    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    for (int i = 0; i < N_timesteps - 1; i++)
    {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }
    return result;
}

// Takes in coeffs and x point to calculate y point on polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for(int i=0; i<coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x,i);
    }
    return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size()==yvals.size());
    assert(order>= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXd A(xvals.size(), order+1);

    for(int i=0; i < xvals.size(); i++)
    {
        A(i,0) = 1.0;
    }

    for(int j=0; j < xvals.size(); j++)
    {
        for(int i=0; i<order; i++)
        {
            A(j, i+1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

class Trajectory{
    public:
        ros::NodeHandle nh;

    private:

        ros::Subscriber traj_sub;
        ros::Publisher traj_pub;
        ros::Publisher closestPointMarker_pub;
        ros::Publisher closestPathMarker_pub;
        ros::Publisher predictedPathMarker_pub;

        ros::Subscriber amclpose_sub;
        ros::Subscriber odom_sub;

        nav_msgs::Path globalPath;
        nav_msgs::Path trajNormalized;
        nav_msgs::Path trajCalculated;

        geometry_msgs::Pose poseCar;
        std::vector<double> rpyCar;

        Eigen::VectorXd localCoeffs;
        bool coeffsLocalSet = false;

        double velocity;

        MPC mpc;

        visualization_msgs::Marker closestPointMarker;
        visualization_msgs::MarkerArray closestPathMarker;
        visualization_msgs::MarkerArray predictedPathMarker;

    public:

        Trajectory()
        {
            mpc = MPC();

            traj_sub = nh.subscribe<nav_msgs::Path>("/trajectory",10, &Trajectory::trajCallback, this);
            traj_pub = nh.advertise<nav_msgs::Path>("/trajectory_local",1);
            closestPointMarker_pub = nh.advertise<visualization_msgs::Marker>("/closestPoint",10);
            closestPathMarker_pub = nh.advertise<visualization_msgs::MarkerArray>("/closestPath",10);
            predictedPathMarker_pub = nh.advertise<visualization_msgs::MarkerArray>("/predictedPath",10);

            amclpose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10, &Trajectory::carPoseCallback, this);
            odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom",10,&Trajectory::odomCallback,this);

        }
        ~Trajectory()
        {
            ROS_INFO("trajectory saver bye bye");
        }

        void trajCallback(const nav_msgs::Path::ConstPtr &trajectoryMsg)
        {            
            // Get the global trajectory and set
            if(!globalPath.poses.size()>0)
            {
                ROS_INFO("Retrieving global trajectory...");
                globalPath = *trajectoryMsg;
                ROS_INFO("Retrieved path: %d points ...",globalPath.poses.size());
            }
        }

        void trajectoryPublish()
        {
            // ROS_INFO("publishing: %d",trajNormalized.poses.size());
            traj_pub.publish(trajNormalized);

        }

        void setClosestPointMarker(double pos_x, double pos_y)
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

        void closestPointMarkerPublish()
        {
            closestPointMarker_pub.publish(closestPointMarker);
        }

        void pathMarker(visualization_msgs::MarkerArray &markerArray,std::string frame_, const std::vector<double> pos_x, const std::vector<double> pos_y,double r_ = 0.0, double g_ = 0.0, double b_ = 0.0, double a_ = 1.0)
        {
            for(int id = 0; id < pos_x.size(); id++)
            {
                markerArray.markers[id].header.frame_id = frame_;
                markerArray.markers[id].header.stamp = ros::Time();
                markerArray.markers[id].type = visualization_msgs::Marker::CUBE;
                markerArray.markers[id].action = visualization_msgs::Marker::ADD;
                markerArray.markers[id].id = id;
                markerArray.markers[id].pose.position.x = pos_x[id];
                markerArray.markers[id].pose.position.y = pos_y[id];
                markerArray.markers[id].pose.orientation.x = 0.0;
                markerArray.markers[id].pose.orientation.y = 0.0;
                markerArray.markers[id].pose.orientation.z = 0.0;
                markerArray.markers[id].pose.orientation.w = 1.0;
                markerArray.markers[id].scale.x = 0.1;
                markerArray.markers[id].scale.y = 0.1;
                markerArray.markers[id].scale.z = 0.1;
                markerArray.markers[id].color.a = a_; // Don't forget to set the alpha!
                markerArray.markers[id].color.r = r_;
                markerArray.markers[id].color.g = g_;
                markerArray.markers[id].color.b = b_;   
            }             
        }

        void pathMarkerPublish()
        {
            closestPathMarker_pub.publish(closestPathMarker);
            predictedPathMarker_pub.publish(predictedPathMarker);
        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
        {
            velocity = odomMsg->twist.twist.linear.x;
        }

        void carPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclPoseMsg)
        {
            poseCar = amclPoseMsg->pose.pose;
            // ROS_INFO("********** \n CAR: x: %.2f y: %.2f w: %.2f",poseCar.position.x,poseCar.position.y,poseCar.orientation.w);

            // Car Orientation
            tf::Quaternion q_car(
                poseCar.orientation.x,
                poseCar.orientation.y,
                poseCar.orientation.z,
                poseCar.orientation.w);

            tf::Matrix3x3 m_car(q_car);
            double car_roll,car_pitch,car_yaw;
            m_car.getRPY(car_roll,car_pitch,car_yaw);

            rpyCar.push_back(car_roll);
            rpyCar.push_back(car_pitch);
            rpyCar.push_back(car_yaw);

            nav_msgs::Path cleanedPath;
            if(globalPath.poses.size()>0)
            {
                // cleanedPath = cleanPath(globalPath);
                // coeffsOfCurrentCarPose(getClosestPathPoint(cleanedPath));
                coeffsOfCurrentCarPose(getClosestPathPoint(globalPath));
            }

            if(coeffsLocalSet)
            {
                // ROS_INFO("a %.2f b %.2f c %.2f d %.2f", localCoeffs[0],localCoeffs[1],localCoeffs[2],localCoeffs[3]);

                double poly_inc = 0.1;
                int num_points = 10;

                closestPathMarker.markers.resize(num_points);

                std::vector<double> x_vals;
                std::vector<double> y_vals;
       
                for (int i = 1; i < num_points; i++)
                {
                    x_vals.push_back(poly_inc*i);
                    y_vals.push_back(polyeval(localCoeffs,poly_inc*i));
                }          
                // pathMarker(closestPathMarker, poly_inc*i,polyeval(localCoeffs,poly_inc*i), i); 
                pathMarker(closestPathMarker, "base_link", x_vals, y_vals, 0, 0, 1.0); 
            }
        }

        double calculate_latency_state(double x, double y, double psi, double v, double steer_value, double throttle_value, Eigen::VectorXd coeffs, double latency)
        {

        double dt = latency / 1000; //milisecond to second

        x += x + v * cos(psi) * dt;
        psi -= steer_value / Lf * dt;
        v += throttle_value * dt;
        double epsi = psi - atan(coeffs[1] + 2 * x * coeffs[2] + 3 * coeffs[3] * pow(x, 2));
        double cte = polyeval(coeffs, 0) + v * sin(epsi) * dt;

        return v, cte, epsi;
        }

        void predictControl()
        {
            if(coeffsLocalSet)
            {
                double epsi = 0.0;
                double cte = 0.0;
               )
                epsi = rpyCar[2] -  atan(localCoeffs[1] + 2 * poseCar.position.x * localCoeffs[2] + 3 * localCoeffs[3] * pow(poseCar.position.x, 2));
                cte = polyeval(localCoeffs, 0);// + velocity * sin(epsi);

                Eigen::VectorXd state(6);
                state << 0,0,0,velocity,cte,epsi;

                auto vars = mpc.Solve(state,localCoeffs);

                ROS_INFO("steer: %.2f throttle: %.2f", vars[0], vars[1]);

                std::vector<double> mpc_x_vals;
                std::vector<double> mpc_y_vals;

                for (int i = 2; i < vars.size(); i++)
                {
                    if (i % 2 == 0){
                    mpc_x_vals.push_back(vars[i]);
                    }
                    else{
                    mpc_y_vals.push_back(vars[i]);
                    }                  
                }
                if(mpc_x_vals.size()>0)
                {
                    predictedPathMarker.markers.resize(mpc_x_vals.size());
                    pathMarker(predictedPathMarker, "base_link", mpc_x_vals, mpc_y_vals, 1.0, 1.0);
                }
                    
                // ROS_INFO("x: %.4f y: %.4f cte: %.4f epsi: %.4f",poseCar.position.x, polyeval(localCoeffs, poseCar.position.x), cte ,epsi);
            }
        }

        nav_msgs::Path cleanPath(const nav_msgs::Path globalTrajectory)
        {
            double min_dist = 0.10; //m

            nav_msgs::Path cleanedPath = globalTrajectory;

            ROS_INFO("Min dist btw particles: %.2f m. \n Before cleaning size: %d",min_dist, cleanedPath.poses.size());

            cleanedPath.poses.push_back(globalTrajectory.poses[0]);

            int i = 0;
            int j = 1;
            while(i <globalTrajectory.poses.size()-1)
            {
                double cur_x = globalTrajectory.poses[i].pose.position.x; 
                double next_x = globalTrajectory.poses[j].pose.position.x;
                double dist_x = next_x-cur_x;

                double cur_y = globalTrajectory.poses[i].pose.position.y; 
                double next_y = globalTrajectory.poses[j].pose.position.y;
                double dist_y = next_y-cur_y;

                double dist = sqrt(pow(dist_x,2)+pow(dist_y,2));
                // ROS_INFO("%d-%d dist_x: %.2f dist_y: %.2f -> dist: %.2f",i,j,dist_x,dist_y,dist);
                if(dist < min_dist)
                {
                    // ROS_INFO("removing particle %d", j);
                    cleanedPath.poses.erase(cleanedPath.poses.begin()+j);                    
                }
                else
                {            
                    i = j;                                 
                }
                j++;
            }
            ROS_INFO("After cleaning, size changed to: %d",cleanedPath.poses.size());

            return cleanedPath;
        }

        int getClosestPathPoint(const nav_msgs::Path globalTrajectory)
        {
            nav_msgs::Path closestPath = globalTrajectory;
            double shortest_dist = 1.0e9;
            double smallest_theta = 1.0e9;
            int closest_point = -1;

            // Find closest path point to the car and return a path starting from that point 
            for(int i=0; i<globalTrajectory.poses.size(); i++)
            {
                double dist_x = 0;
                double dist_y = 0;
                double new_dist = 0.0;
                double new_theta = 0.0;

                dist_x = globalTrajectory.poses[i].pose.position.x - poseCar.position.x;
                dist_y = globalTrajectory.poses[i].pose.position.y - poseCar.position.y;

                tf::Pose traj_pose;
                tf::poseMsgToTF (globalTrajectory.poses[i].pose,traj_pose);
                tf::Pose poseCarTf;
                tf::poseMsgToTF (poseCar,poseCarTf);

                tf::Pose diffTF = traj_pose.inverseTimes(poseCarTf);

                geometry_msgs::Transform diffMsg;
                tf::transformTFToMsg (diffTF, diffMsg);          
                // ROS_INFO("translation: x %.2f y %.2f, rotation: z %.2f",diffMsg.translation.x,diffMsg.translation.y, tf::getYaw(diffMsg.rotation) * 180 / 3.1415);

                double dist_xy = sqrt(diffMsg.translation.x*diffMsg.translation.x+diffMsg.translation.y*diffMsg.translation.y);
                double theta_angle = fabs(tf::getYaw(diffMsg.rotation));

                if(dist_xy < shortest_dist)
                {                
                    // shortest_dist = dist_xy;
                    // closest_point = i;    
                    if(fabs(theta_angle) < M_PI/2.0)
                    {
                        shortest_dist = dist_xy;
                        smallest_theta = theta_angle;
                        closest_point = i;    
                    }          
                    
                }                      
            }
            // If closest point found get x,y positions and z rotation
            if(closest_point >= 0)
            {
                double pos_x = globalTrajectory.poses[closest_point].pose.position.x;
                double pos_y = globalTrajectory.poses[closest_point].pose.position.y;
                double rot_z = tf::getYaw(globalTrajectory.poses[closest_point].pose.orientation);
                // ROS_INFO("shortest dist %.2f @ %d x: %.2f y: %.2f z: %.2f",shortest_dist,closest_point, pos_x, pos_y,rot_z);                
                // Show the closest point on RVIZ as a marker
                setClosestPointMarker(pos_x,pos_y);                
            }   

            return closest_point;

        }

        void coeffsOfCurrentCarPose(int point)
        {     
            std::vector<double> ptsx,ptsy;

            // Limit the number of trajectory points to be shifted
            u_int coeff_N = 12;

            if(globalPath.poses.size()!=0)
            {
                // closestPathMarker.markers.resize(coeff_N);
                // Shift Path Points
                for(int i=0; i<coeff_N; i++)
                {         
                  // Shift
                    double shift_x= globalPath.poses[point + i].pose.position.x - poseCar.position.x;
                    double shift_y= globalPath.poses[point + i].pose.position.y - poseCar.position.y;       
                    // Rotate
                    ptsx.push_back((shift_x * cos(0 - rpyCar[2])) - (shift_y * sin(0 - rpyCar[2])));
                    ptsy.push_back((shift_x * sin(0 - rpyCar[2])) + (shift_y * cos(0 - rpyCar[2])));                
                }

                // ROS_INFO("trajNormalized.poses.size(): %d",shiftedPath.poses.size());
                double *ptrx = &ptsx[0];
                Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx,coeff_N);

                double *ptry = &ptsy[0];
                Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry,coeff_N);

                auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

                // ROS_INFO("a: %.2f b: %.2f c: %.2f d: %.2f",coeffs[0],coeffs[1],coeffs[2],coeffs[3]);

                localCoeffs = coeffs;
                coeffsLocalSet = true;
            }
        }

        double calc_errors()
        {
            double dt = 0.1;
            double x = poseCar.position.x;

            double epsi = rpyCar[2] -  atan(localCoeffs[1] + 2 * x * localCoeffs[2] + 3 * localCoeffs[3] * pow(x, 2));

            double cte = polyeval(localCoeffs, 0) + 0.2 * sin(epsi);

            ROS_INFO("cte: %.2f epsi: %.2f",cte,epsi);
            return cte,epsi;
        }
    };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc");

    // Instantiate Classes
    Trajectory trj;

    ros::Rate rate(10);

    double cte,epsi;

    while(ros::ok())
    {
        trj.trajectoryPublish();
        trj.closestPointMarkerPublish();
        trj.pathMarkerPublish();

        trj.predictControl();

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}