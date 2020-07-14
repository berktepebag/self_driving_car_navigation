#include <iostream>

#include "rc_car_2dnav/path_utilities.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc");

    // Instantiate Classes
    MPC mpc;
    Trajectory trj(mpc);

    ros::Rate rate(50);

    double cte, epsi;

    while (ros::ok())
    {
        trj.closestPointMarkerPublish();
        trj.pathMarkerPublish();

        trj.predictControl();
        trj.cmdPublish();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}